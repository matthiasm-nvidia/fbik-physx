#define NOMINMAX
#include "MovieRecorder.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <windows.h>

namespace
{
struct Handle
{
    HANDLE value = nullptr;
    ~Handle()
    {
        close();
    }
    void close()
    {
        if (value && value != INVALID_HANDLE_VALUE)
            CloseHandle(value);
        value = nullptr;
    }
};
// Windows argv quoting, without a command shell (including spaces, Unicode, and trailing slashes).
std::wstring quote(const std::wstring &value)
{
    std::wstring result = L"\"";
    size_t slashes = 0;
    for (wchar_t c : value)
    {
        if (c == L'\\')
        {
            ++slashes;
            continue;
        }
        result.append(c == L'"' ? slashes * 2 + 1 : slashes, L'\\');
        result += c;
        slashes = 0;
    }
    result.append(slashes * 2, L'\\');
    return result + L"\"";
}
} // namespace
struct MovieRecorder::Impl
{
    Handle process, input;
    std::filesystem::path log;
    std::string path;
    int width = 0, height = 0, fps = 30;
    size_t frames = 0;
    std::string finish()
    {
        input.close();
        if (!process.value)
            return {};
        DWORD exitCode = 1;
        const DWORD wait = WaitForSingleObject(process.value, 30000);
        if (wait != WAIT_OBJECT_0)
        {
            TerminateProcess(process.value, 1);
            WaitForSingleObject(process.value, INFINITE);
        }
        GetExitCodeProcess(process.value, &exitCode);
        process.close();
        std::ifstream stream(log, std::ios::binary);
        std::string detail((std::istreambuf_iterator<char>(stream)), {});
        stream.close();
        std::error_code ignored;
        std::filesystem::remove(log, ignored);
        log.clear();
        if (detail.size() > 4000)
            detail = detail.substr(detail.size() - 4000);
        if (wait != WAIT_OBJECT_0)
            return "FFmpeg did not finish within 30 seconds. " + detail;
        if (exitCode)
            return "FFmpeg could not finish the movie. " + detail;
        return {};
    }
    ~Impl()
    {
        finish();
        if (!log.empty())
        {
            std::error_code ignored;
            std::filesystem::remove(log, ignored);
        }
    }
};
MovieRecorder::MovieRecorder() : impl(std::make_unique<Impl>()) {}
MovieRecorder::~MovieRecorder() = default;
bool MovieRecorder::active() const
{
    return impl->process.value != nullptr;
}
int MovieRecorder::width() const
{
    return impl->width;
}
int MovieRecorder::height() const
{
    return impl->height;
}
int MovieRecorder::fps() const
{
    return impl->fps;
}
size_t MovieRecorder::frames() const
{
    return impl->frames;
}
const std::string &MovieRecorder::path() const
{
    return impl->path;
}
void MovieRecorder::start(const std::string &path, int width, int height, int fps, bool bottomUp)
{
    if (active())
        throw std::runtime_error("A movie is already recording.");
    if (width <= 0 || height <= 0 || width > 16384 || height > 16384 || (fps != 30 && fps != 60))
        throw std::runtime_error("Invalid recording dimensions or frame rate.");
    wchar_t executable[32768];
    const DWORD found = SearchPathW(nullptr, L"ffmpeg.exe", nullptr, 32768, executable, nullptr);
    if (!found || found >= 32768)
        throw std::runtime_error("FFmpeg was not found. Install ffmpeg.exe and add its folder to PATH.");
    auto next = std::make_unique<Impl>();
    next->width = width;
    next->height = height;
    next->fps = fps;
    next->path = path;
    wchar_t tempDirectory[MAX_PATH], tempFile[MAX_PATH];
    const DWORD tempLength = GetTempPathW(MAX_PATH, tempDirectory);
    if (!tempLength || tempLength >= MAX_PATH || !GetTempFileNameW(tempDirectory, L"fbm", 0, tempFile))
        throw std::runtime_error("Could not create the FFmpeg diagnostic log.");
    next->log = tempFile;
    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    Handle readPipe, log;
    if (!CreatePipe(&readPipe.value, &next->input.value, &security, 0) ||
        !SetHandleInformation(next->input.value, HANDLE_FLAG_INHERIT, 0))
        throw std::runtime_error("Could not create the FFmpeg input pipe.");
    log.value = CreateFileW(tempFile, GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &security, CREATE_ALWAYS,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
    if (log.value == INVALID_HANDLE_VALUE)
        throw std::runtime_error("Could not open the FFmpeg diagnostic log.");
    STARTUPINFOEXW startup{};
    startup.StartupInfo.cb = sizeof(startup);
    startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdInput = readPipe.value;
    startup.StartupInfo.hStdOutput = startup.StartupInfo.hStdError = log.value;
    SIZE_T attributeSize = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &attributeSize);
    std::vector<uint8_t> storage(attributeSize);
    startup.lpAttributeList = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(storage.data());
    if (!InitializeProcThreadAttributeList(startup.lpAttributeList, 1, 0, &attributeSize))
        throw std::runtime_error("Could not initialize FFmpeg process attributes.");
    HANDLE inherited[] = {readPipe.value, log.value};
    const bool attributes = UpdateProcThreadAttribute(startup.lpAttributeList, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST,
                                                      inherited, sizeof(inherited), nullptr, nullptr) != FALSE;
    const std::wstring filter = bottomUp ? L"vflip,pad=ceil(iw/2)*2:ceil(ih/2)*2" : L"pad=ceil(iw/2)*2:ceil(ih/2)*2";
    std::wstring command =
        quote(executable) + L" -hide_banner -loglevel error -nostdin -y -f rawvideo -pixel_format rgba -video_size " +
        std::to_wstring(width) + L"x" + std::to_wstring(height) + L" -framerate " + std::to_wstring(fps) +
        L" -i pipe:0 -an -c:v libx264 -preset fast -crf 21 -pix_fmt yuv420p -vf " + quote(filter) +
        L" -movflags +faststart -f mp4 " + quote(std::filesystem::u8path(path).wstring());
    PROCESS_INFORMATION process{};
    const bool created = attributes && CreateProcessW(executable, command.data(), nullptr, nullptr, TRUE,
                                                      CREATE_NO_WINDOW | EXTENDED_STARTUPINFO_PRESENT, nullptr, nullptr,
                                                      &startup.StartupInfo, &process);
    const DWORD createError = GetLastError();
    DeleteProcThreadAttributeList(startup.lpAttributeList);
    if (!created)
        throw std::runtime_error("Could not start FFmpeg (Windows error " + std::to_string(createError) + ").");
    next->process.value = process.hProcess;
    CloseHandle(process.hThread);
    impl = std::move(next);
}
void MovieRecorder::write(const uint8_t *rgba, size_t bytes)
{
    if (!active() || !rgba || bytes != size_t(width()) * height() * 4)
        throw std::runtime_error("Recording frame size does not match the movie.");
    size_t offset = 0;
    while (offset < bytes)
    {
        DWORD written = 0;
        if (!WriteFile(impl->input.value, rgba + offset, DWORD(std::min(bytes - offset, size_t(1 << 20))), &written,
                       nullptr) ||
            !written)
        {
            const auto error = impl->finish();
            throw std::runtime_error(error.empty() ? "FFmpeg stopped accepting frames." : error);
        }
        offset += written;
    }
    ++impl->frames;
}
void MovieRecorder::stop()
{
    const auto error = impl->finish();
    if (!error.empty())
        throw std::runtime_error(error);
}
