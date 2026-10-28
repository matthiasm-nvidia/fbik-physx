#define NOMINMAX
#include <windows.h>
#include <shobjidl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include "FileDialog.h"
#include <filesystem>
#include <stdexcept>

static std::string selectFile(GLFWwindow *owner, bool saveMovie)
{
    const HRESULT initialized = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    if (FAILED(initialized) && initialized != RPC_E_CHANGED_MODE)
        throw std::runtime_error("Could not initialize the file dialog.");
    IFileDialog *dialog = nullptr;
    HRESULT result = CoCreateInstance(saveMovie ? CLSID_FileSaveDialog : CLSID_FileOpenDialog, nullptr,
                                      CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&dialog));
    std::string selected;
    if (SUCCEEDED(result))
    {
        const COMDLG_FILTERSPEC filters[] = {{L"USD stages", L"*.usd;*.usda;*.usdc;*.usdz"}};
        const COMDLG_FILTERSPEC movies[] = {{L"MP4 movie", L"*.mp4"}};
        dialog->SetFileTypes(1, saveMovie ? movies : filters);
        dialog->SetTitle(saveMovie ? L"Record MP4 movie" : L"Load USD stage");
        if (saveMovie)
        {
            dialog->SetDefaultExtension(L"mp4");
            dialog->SetFileName(L"recording.mp4");
        }
        DWORD options = 0;
        dialog->GetOptions(&options);
        dialog->SetOptions(options | (saveMovie ? FOS_OVERWRITEPROMPT : FOS_FILEMUSTEXIST) |
                           FOS_PATHMUSTEXIST | FOS_FORCEFILESYSTEM);
        result = dialog->Show(glfwGetWin32Window(owner));
        if (SUCCEEDED(result))
        {
            IShellItem *item = nullptr;
            result = dialog->GetResult(&item);
            if (SUCCEEDED(result))
            {
                PWSTR path = nullptr;
                result = item->GetDisplayName(SIGDN_FILESYSPATH, &path);
                if (SUCCEEDED(result))
                {
                    selected = std::filesystem::path(path).u8string();
                    CoTaskMemFree(path);
                }
                item->Release();
            }
        }
        dialog->Release();
    }
    if (SUCCEEDED(initialized))
        CoUninitialize();
    if (FAILED(result) && result != HRESULT_FROM_WIN32(ERROR_CANCELLED))
        throw std::runtime_error("The file dialog failed.");
    return selected;
}

std::string selectUsdFile(GLFWwindow *owner) { return selectFile(owner, false); }
std::string selectMovieFile(GLFWwindow *owner) { return selectFile(owner, true); }
