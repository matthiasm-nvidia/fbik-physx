#define NOMINMAX
#include "Camera.h"
#include "FileDialog.h"
#include "RtxView.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "viewer_imgui_theme.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <string>
#include <windows.h>
#include "RecentFiles.h"

namespace
{
struct Viewer
{
    viewer::Camera camera;
    viewer::Vec3 orbitCenter{0, 0, 0};
    viewer::Vec3 sceneCenter{0, 0, 0};
    float sceneRadius = 5;
    bool zUp = false;
    int mouseX = 0, mouseY = 0;
    std::string pendingLoad;
};
Viewer &state(GLFWwindow *window)
{
    return *static_cast<Viewer *>(glfwGetWindowUserPointer(window));
}
void frameScene(Viewer &v)
{
    v.orbitCenter = v.sceneCenter;
    v.camera.orthographic = false;
    v.camera.horizontalOffset = v.camera.verticalOffset = 0;
    v.camera.nearClip = std::max(0.0001f, v.sceneRadius * 0.00001f);
    v.camera.farClip = std::max(1000.0f, v.sceneRadius * 1000);
    v.camera.orbitUp = v.zUp ? viewer::Vec3(0, 0, 1) : viewer::Vec3(0, 1, 0);
    auto direction = v.zUp ? viewer::Vec3(1, -1, 0.65f) : viewer::Vec3(1, 0.65f, 1);
    direction.normalize();
    const float distance = v.sceneRadius / std::sin(v.camera.fov * 3.14159265f / 360) * 1.15f;
    v.camera.look_at(v.orbitCenter + direction * distance, v.orbitCenter);
    v.camera.right = v.camera.forward.cross(v.camera.orbitUp);
    v.camera.right.normalize();
    v.camera.up = v.camera.right.cross(v.camera.forward);
    v.camera.up.normalize();
    v.camera.speed = std::max(0.0001f, v.sceneRadius * 0.05f);
}
void keyCallback(GLFWwindow *w, int key, int scancode, int action, int mods)
{
    if (key < 0 || key >= static_cast<int>(std::size(ImGui::GetIO().KeysDown)))
        return;
    ImGui_ImplGLFW_KeyCallback(w, key, scancode, action, mods);
    if (ImGui::GetIO().WantCaptureKeyboard)
        return;
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_ESCAPE)
            glfwSetWindowShouldClose(w, GLFW_TRUE);
        else if (key == GLFW_KEY_HOME)
            frameScene(state(w));
        else
            state(w).camera.handle_key_down(key);
    }
}
void mouseCallback(GLFWwindow *w, int button, int action, int mods)
{
    ImGui_ImplGLFW_MouseButtonCallback(w, button, action, mods);
    double x, y;
    glfwGetCursorPos(w, &x, &y);
    state(w).mouseX = static_cast<int>(x);
    state(w).mouseY = static_cast<int>(y);
}
void scrollCallback(GLFWwindow *w, double x, double y)
{
    ImGui_ImplGLFW_ScrollCallback(w, x, y);
    if (!ImGui::GetIO().WantCaptureMouse)
        state(w).camera.handle_wheel(static_cast<float>(y));
}
void cursorCallback(GLFWwindow *w, double x, double y)
{
    auto &v = state(w);
    const int dx = static_cast<int>(x) - v.mouseX, dy = static_cast<int>(y) - v.mouseY;
    v.mouseX = static_cast<int>(x);
    v.mouseY = static_cast<int>(y);
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
        v.camera.handle_mouse_view(dx, dy);
    else if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
    {
        const auto before = v.camera.pos;
        v.camera.handle_mouse_translate(dx, dy,
                                        std::max(0.00001f, (v.camera.pos - v.orbitCenter).magnitude() * 0.001f));
        v.orbitCenter += v.camera.pos - before;
    }
    else if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        v.camera.handle_mouse_orbit(dx, dy, v.orbitCenter);
}
void drawImage(GLuint texture, int width, int height, const std::vector<uint8_t> &pixels)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 1);
    glVertex2f(-1, -1);
    glTexCoord2f(1, 1);
    glVertex2f(1, -1);
    glTexCoord2f(1, 0);
    glVertex2f(1, 1);
    glTexCoord2f(0, 0);
    glVertex2f(-1, 1);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}
constexpr const char *usage = "Usage: fbik_viewer [--scene file.usd] [--help]\n";
} // namespace

int wmain(int argc, wchar_t **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::string scenePath;
    try
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::wstring arg = argv[i];
            if (arg == L"--help" || arg == L"-h")
            {
                std::puts(usage);
                return 0;
            }
            if (arg == L"--scene" && i + 1 < argc)
                scenePath = std::filesystem::absolute(argv[++i]).u8string();
            else
                throw std::runtime_error("Unknown argument or missing --scene path.");
        }
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n%s", e.what(), usage);
        return 1;
    }
    wchar_t executablePath[32768];
    const DWORD length = GetModuleFileNameW(nullptr, executablePath, 32768);
    if (!length || length == 32768)
        return 1;
    const auto executableDirectory = std::filesystem::path(executablePath).parent_path();
    viewer::RecentFiles recent(executableDirectory / "../../.cache/recent-usd.txt");

    glfwSetErrorCallback([](int code, const char *message) { std::fprintf(stderr, "GLFW %d: %s\n", code, message); });
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    viewer::imgui_theme::init_ui_scale();
    const float scale = viewer::imgui_theme::ui_scale();
    GLFWwindow *window = glfwCreateWindow(int(1600 * scale), int(1000 * scale), "FBIK PhysX - USD Viewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    ImGuiContext *context = ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    viewer::imgui_theme::apply_kit_dark_style();
    viewer::imgui_theme::load_ui_font(window);
    ImGui_ImplGLFW_Init(window, false);
    Viewer v;
    v.camera.init();
    frameScene(v);
    v.pendingLoad = scenePath;
    glfwSetWindowUserPointer(window, &v);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCharCallback(window, ImGui_ImplGLFW_CharCallback);
    glfwSetMouseButtonCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetCursorPosCallback(window, cursorCallback);
    GLuint texture = 0;
    glGenTextures(1, &texture);
    int exitCode = 0;
    try
    {
        viewer::RtxView rtx;
        std::string error;
        bool renderFailed = false;
        double previousTime = glfwGetTime(), accumulator = 0;
        while (!glfwWindowShouldClose(window))
        {
            glfwPollEvents();
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            if (width <= 0 || height <= 0)
            {
                previousTime = glfwGetTime();
                accumulator = 0;
                glfwWaitEventsTimeout(0.05);
                continue;
            }
            const double now = glfwGetTime();
            const double elapsed = std::min(now - previousTime, 0.1);
            previousTime = now;
            if (rtx.simulating() && !renderFailed)
            {
                accumulator += elapsed;
                try
                {
                    while (accumulator >= 1.0 / 60.0)
                    {
                        rtx.stepSimulation();
                        accumulator -= 1.0 / 60.0;
                    }
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    try { rtx.stopSimulation(); }
                    catch (const std::exception &reset) { error += std::string("\nReset failed: ") + reset.what(); renderFailed = true; }
                    accumulator = 0;
                }
            }
            glViewport(0, 0, width, height);
            glClearColor(0.055f, 0.065f, 0.075f, 1);
            glClear(GL_COLOR_BUFFER_BIT);
            if (rtx.loaded() && !renderFailed)
            {
                try
                {
                    rtx.render(v.camera, width, height);
                    drawImage(texture, rtx.width(), rtx.height(), rtx.pixels());
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    renderFailed = true;
                    std::fprintf(stderr, "%s\n", e.what());
                }
            }
            ImGui_ImplGLFW_NewFrame();
            const auto ui = viewer::imgui_theme::ui;
            ImGui::SetNextWindowPos(ImVec2(ui(16), ui(16)), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(ui(420), ui(260)), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSizeConstraints(ImVec2(ui(420), 0), ImVec2(ui(420), FLT_MAX));
            ImGui::Begin("USD Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            if (ImGui::Button("Load USD..."))
            {
                try
                {
                    const auto selected = selectUsdFile(window);
                    previousTime = glfwGetTime();
                    accumulator = 0;
                    if (!selected.empty())
                        v.pendingLoad = selected;
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Recent USD"))
                ImGui::OpenPopup("Recent USD files");
            if (ImGui::BeginPopup("Recent USD files"))
            {
                if (recent.paths().empty())
                    ImGui::TextDisabled("No recent files");
                for (const auto &path : recent.paths())
                {
                    ImGui::PushID(path.c_str());
                    const auto label = std::filesystem::u8path(path).filename().u8string();
                    if (ImGui::MenuItem(label.c_str()))
                        v.pendingLoad = path;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", path.c_str());
                    ImGui::PopID();
                }
                ImGui::EndPopup();
            }
            if (ImGui::Button("Load falling box"))
                v.pendingLoad = (executableDirectory / "resources/scenes/falling_box.usda").u8string();
            if (rtx.loaded())
            {
                ImGui::TextWrapped("%s", rtx.path().c_str());
                if (ImGui::Button("Frame stage"))
                    frameScene(v);
                if (ImGui::Button(rtx.simulating() ? "Stop simulation" : "Start simulation"))
                {
                    try
                    {
                        if (rtx.simulating()) rtx.stopSimulation();
                        else if (!renderFailed) rtx.startSimulation();
                        error.clear();
                    }
                    catch (const std::exception &e) { error = e.what(); }
                    accumulator = 0;
                    previousTime = glfwGetTime();
                }
            }
            else
                ImGui::TextWrapped("Load a USD file to view it.");
            if (!v.pendingLoad.empty())
                ImGui::TextWrapped("Loading USD... First RTX initialization may take several minutes.");
            if (!error.empty())
                ImGui::TextWrapped("%s", error.c_str());
            ImGui::Separator();
            ImGui::TextWrapped("Left drag: orbit | Right drag: look | Middle drag: pan\nWheel: move | W/A/S/D, Q/E: move | Home: frame");
            ImGui::End();
            ImGui::Render();
            ImGui_ImplGLFW_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);

            // Present loading feedback before synchronous SDK initialization and population.
            if (!v.pendingLoad.empty())
            {
                const auto requested = v.pendingLoad;
                v.pendingLoad.clear();
                glfwSetWindowTitle(window, "FBIK PhysX - Loading USD...");
                try
                {
                    if (!std::filesystem::is_regular_file(std::filesystem::u8path(requested)))
                        throw std::runtime_error("USD file does not exist: " + requested);
                    rtx.initialize(executableDirectory.u8string());
                    rtx.load(requested, width, height);
                    renderFailed = false;
                    v.sceneCenter = v.orbitCenter = rtx.center();
                    v.sceneRadius = rtx.radius();
                    v.zUp = rtx.zUp();
                    v.camera.init();
                    if (const auto *authored = rtx.initialCamera())
                    {
                        v.camera = *authored;
                        v.camera.orbitUp = v.zUp ? viewer::Vec3(0, 0, 1) : viewer::Vec3(0, 1, 0);
                        v.camera.speed = std::max(0.0001f, v.sceneRadius * 0.05f);
                        const float depth = (v.orbitCenter - v.camera.pos).dot(v.camera.forward);
                        v.orbitCenter = v.camera.pos + v.camera.forward * std::max(depth, v.sceneRadius);
                    }
                    else
                        frameScene(v);
                    error.clear();
                    recent.add(requested);
                    recent.save();
                    std::printf("Loaded USD: %s\n", requested.c_str());
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    std::fprintf(stderr, "%s\n", e.what());
                }
                const auto title = rtx.loaded() ? "FBIK PhysX - " + std::filesystem::u8path(rtx.path()).filename().u8string()
                                                : "FBIK PhysX - USD Viewer";
                glfwSetWindowTitle(window, title.c_str());
                previousTime = glfwGetTime();
                accumulator = 0;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n", e.what());
        exitCode = 1;
    }
    glDeleteTextures(1, &texture);
    ImGui_ImplGLFW_Shutdown();
    ImGui::DestroyContext(context);
    glfwDestroyWindow(window);
    glfwTerminate();
    return exitCode;
}
