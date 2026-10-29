#define NOMINMAX
#include "Camera.h"
#include "FileDialog.h"
#include "MovieRecorder.h"
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
#include <fstream>
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
    std::string pendingLoad, inputError;
    viewer::RtxView *rtx = nullptr;
    bool playing = false, animationPlaying = false, loop = true, follow = false;
    float playbackSpeed = 1;
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
    v.camera.look_at(v.orbitCenter + direction * (distance * 0.5f), v.orbitCenter);
    v.camera.right = v.camera.forward.cross(v.camera.orbitUp);
    v.camera.right.normalize();
    v.camera.up = v.camera.right.cross(v.camera.forward);
    v.camera.up.normalize();
    v.camera.speed = std::max(0.0001f, v.sceneRadius * 0.05f);
}
void frameCharacter(Viewer &v)
{
    if (!v.rtx || !v.rtx->characterJointCount()) return;
    Viewer focused = v;
    focused.sceneCenter = v.rtx->characterCenter();
    focused.sceneRadius = v.rtx->characterRadius();
    frameScene(focused);
    v.camera = focused.camera;
    v.orbitCenter = focused.orbitCenter;
}
void preparePlayback(Viewer &v)
{
    if (!v.rtx || !v.rtx->loaded()) return;
    v.rtx->startSimulation();
    v.rtx->setAnimationLoop(v.loop);
    if (v.animationPlaying) v.rtx->startAnimation();
    else v.rtx->stopAnimation();
}
void sameLineIfFits(const char *label, bool checkbox = false)
{
    const auto &style = ImGui::GetStyle();
    const float width = ImGui::CalcTextSize(label).x +
        (checkbox ? ImGui::GetFontSize() + 2 * style.FramePadding.y + style.ItemInnerSpacing.x : 2 * style.FramePadding.x);
    if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + width <=
        ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x)
        ImGui::SameLine();
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
        else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
        {
            auto &v = state(w);
            try
            {
                if (v.rtx && v.rtx->loaded())
                {
                    if (!v.playing) preparePlayback(v);
                    else v.rtx->endDrag();
                    v.playing = !v.playing;
                }
            }
            catch (const std::exception &e) { v.inputError = e.what(); }
        }
        else
            state(w).camera.handle_key_down(key);
    }
}
viewer::Camera::Ray cursorRay(GLFWwindow *w, double x, double y)
{
    int width, height;
    glfwGetWindowSize(w, &width, &height); // Cursor and dimensions use the same logical pixels.
    return state(w).camera.screenRay(float(x), float(y), float(width), float(height));
}
void focusCallback(GLFWwindow *w, int focused)
{
    if (!focused && state(w).rtx)
        try { state(w).rtx->endDrag(); }
        catch (const std::exception &e) { state(w).inputError = e.what(); }
}
void mouseCallback(GLFWwindow *w, int button, int action, int mods)
{
    ImGui_ImplGLFW_MouseButtonCallback(w, button, action, mods);
    double x, y;
    glfwGetCursorPos(w, &x, &y);
    state(w).mouseX = static_cast<int>(x);
    state(w).mouseY = static_cast<int>(y);
    auto &v = state(w);
    if (button == GLFW_MOUSE_BUTTON_LEFT && v.rtx)
    {
        try
        {
            if (action == GLFW_RELEASE) v.rtx->endDrag(); // Release also over ImGui.
            else if (action == GLFW_PRESS && (mods & GLFW_MOD_SHIFT) && !ImGui::GetIO().WantCaptureMouse)
            {
                if (v.rtx->beginDrag(cursorRay(w, x, y))) v.playing = true;
            }
        }
        catch (const std::exception &e) { v.inputError = e.what(); }
    }
}
void scrollCallback(GLFWwindow *w, double x, double y)
{
    ImGui_ImplGLFW_ScrollCallback(w, x, y);
    if (!ImGui::GetIO().WantCaptureMouse && !(state(w).rtx && state(w).rtx->dragging()))
        state(w).camera.handle_wheel(static_cast<float>(y));
}
void cursorCallback(GLFWwindow *w, double x, double y)
{
    auto &v = state(w);
    const int dx = static_cast<int>(x) - v.mouseX, dy = static_cast<int>(y) - v.mouseY;
    v.mouseX = static_cast<int>(x);
    v.mouseY = static_cast<int>(y);
    if (v.rtx && v.rtx->dragging())
    {
        try { v.rtx->moveDrag(cursorRay(w, x, y)); }
        catch (const std::exception &e) { v.inputError = e.what(); }
        return;
    }
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
void drawImage(GLuint texture, int &textureWidth, int &textureHeight,
               int width, int height, const std::vector<uint8_t> &pixels)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (width != textureWidth || height != textureHeight) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        textureWidth = width; textureHeight = height;
    }
    else glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
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
constexpr const char *usage = "Usage: fbik_viewer [--scene file.usd] [--frames count] [--screenshot image.ppm] [--help]\n";
} // namespace

int wmain(int argc, wchar_t **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::string scenePath;
    std::filesystem::path screenshotPath;
    int frameLimit = 0;
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
            else if (arg == L"--frames" && i + 1 < argc)
            {
                const std::wstring value = argv[++i];
                size_t used = 0;
                frameLimit = std::stoi(value, &used);
                if (used != value.size() || frameLimit <= 0) throw std::runtime_error("Invalid frame count.");
            }
            else if (arg == L"--screenshot" && i + 1 < argc)
                screenshotPath = std::filesystem::absolute(argv[++i]);
            else
                throw std::runtime_error("Unknown argument or missing value.");
        }
        if (!screenshotPath.empty() && !frameLimit) frameLimit = 8;
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
    const std::string demoPaths[] = {
        (executableDirectory / "resources/scenes/basic_walk.usda").u8string(),
        (executableDirectory / "resources/scenes/falling_box.usda").u8string()};
    const char *demoNames[] = {"Basic walk", "Falling box"};
    static_assert(std::size(demoPaths) == std::size(demoNames));
    viewer::RecentFiles recent(executableDirectory / "../../.cache/recent-usd.txt");

    glfwSetErrorCallback([](int code, const char *message) { std::fprintf(stderr, "GLFW %d: %s\n", code, message); });
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    if (frameLimit) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
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
    glfwSetWindowFocusCallback(window, focusCallback);
    glfwSetDropCallback(window, [](GLFWwindow *w, int count, const char **paths) {
        if (count > 0) state(w).pendingLoad = paths[0];
    });
    GLuint texture = 0;
    int textureWidth = 0, textureHeight = 0;
    glGenTextures(1, &texture);
    int exitCode = 0;
    try
    {
        viewer::RtxView rtx;
        v.rtx = &rtx;
        std::string error;
        bool renderFailed = false;
        double previousTime = glfwGetTime(), accumulator = 0;
        viewer::Vec3 followCenter{0, 0, 0};
        MovieRecorder recorder;
        std::string recordingStatus;
        bool recordingSavedPopupPending = false;
        double recordingSavedPopupUntil = 0;
        int frames = 0;
        const auto stopRecording = [&] {
            if (!recorder.active()) return;
            const auto path = recorder.path();
            recorder.stop();
            recordingStatus = "Saved " + path;
            recordingSavedPopupPending = true;
        };
        while (!glfwWindowShouldClose(window))
        {
            glfwPollEvents();
            if (!v.inputError.empty())
            {
                error = v.inputError;
                v.inputError.clear();
                v.playing = false;
                try { rtx.stopSimulation(); }
                catch (const std::exception &e) { error += std::string("\n") + e.what(); renderFailed = true; }
            }
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
            if (recorder.active() && (width != recorder.width() || height != recorder.height()))
                try { stopRecording(); }
                catch (const std::exception &e) { error = e.what(); }
            if (v.playing && rtx.simulating() && !renderFailed)
            {
                accumulator += (recorder.active() ? (recorder.frames() ? 1.0 / recorder.fps() : 0) : elapsed) * v.playbackSpeed;
                try
                {
                    while (accumulator >= 1.0 / 60.0)
                    {
                        rtx.stepSimulation();
                        accumulator -= 1.0 / 60.0;
                    }
                    v.animationPlaying = rtx.animating();
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    v.playing = false;
                    try { rtx.stopSimulation(); }
                    catch (const std::exception &reset) { error += std::string("\nReset failed: ") + reset.what(); renderFailed = true; }
                    accumulator = 0;
                }
            }
            else accumulator = 0;
            if (rtx.characterJointCount())
            {
                const auto center = rtx.characterCenter();
                if (v.follow)
                {
                    v.camera.pos += center - followCenter;
                    v.orbitCenter += center - followCenter;
                }
                followCenter = center;
            }
            glViewport(0, 0, width, height);
            glClearColor(0.055f, 0.065f, 0.075f, 1);
            glClear(GL_COLOR_BUFFER_BIT);
            if (rtx.loaded() && !renderFailed)
            {
                try
                {
                    rtx.render(v.camera, width, height);
                    drawImage(texture, textureWidth, textureHeight, rtx.width(), rtx.height(), rtx.pixels());
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    renderFailed = true;
                    v.playing = false;
                    std::fprintf(stderr, "%s\n", e.what());
                }
            }
            ImGui_ImplGLFW_NewFrame();
            const auto ui = viewer::imgui_theme::ui;
            ImGui::SetNextWindowPos(ImVec2(ui(16), ui(16)), ImGuiCond_FirstUseEver);
            const float panelWidth = 330 * 0.7f * scale;
            ImGui::SetNextWindowSizeConstraints(ImVec2(panelWidth, 0), ImVec2(panelWidth, FLT_MAX));
            const auto sceneName = rtx.loaded()
                ? "FBIK Viewer - " + std::filesystem::u8path(rtx.path()).filename().u8string()
                : std::string("FBIK Viewer");
            const auto panelTitle = sceneName + "###FBIK Viewer";
            ImGui::Begin(panelTitle.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            if (ImGui::Button("Load USD..."))
            {
                try
                {
                    rtx.endDrag();
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
            if (ImGui::Button("Recent"))
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
            int selectedDemo = -1;
            for (int i = 0; i < int(std::size(demoPaths)); ++i)
                if (rtx.loaded() && std::filesystem::u8path(rtx.path()) == std::filesystem::u8path(demoPaths[i]))
                    selectedDemo = i;
            ImGui::PushItemWidth(-1);
            if (ImGui::Combo("##Scene selection", &selectedDemo, demoNames, int(std::size(demoNames))))
                v.pendingLoad = demoPaths[selectedDemo];
            ImGui::PopItemWidth();
            if (rtx.loaded())
            {
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", rtx.path().c_str());
                if (ImGui::Button("Reload")) v.pendingLoad = rtx.path();
                ImGui::Separator();
                const bool hasCharacter = rtx.characterJointCount() > 0;
                if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    try
                    {
                        if (ImGui::Button(v.playing ? "Pause simulation" : "Start simulation"))
                        {
                            if (!v.playing && !renderFailed) preparePlayback(v);
                            else rtx.endDrag();
                            v.playing = !v.playing && !renderFailed;
                            previousTime = glfwGetTime();
                            accumulator = 0;
                        }
                        sameLineIfFits("Reset");
                        if (ImGui::Button("Reset"))
                        {
                            rtx.stopSimulation();
                            v.animationPlaying = false;
                            if (v.playing && !renderFailed) preparePlayback(v);
                            accumulator = 0;
                            previousTime = glfwGetTime();
                        }
                        sameLineIfFits("Step");
                        if (ImGui::Button("Step") && !renderFailed)
                        {
                            v.playing = false;
                            rtx.endDrag();
                            preparePlayback(v);
                            rtx.stepSimulation();
                            v.animationPlaying = rtx.animating();
                            accumulator = 0;
                        }
                        if (hasCharacter)
                        {
                            int mode=static_cast<int>(rtx.characterMode());
                            ImGui::PushItemWidth(210 * 0.7f * scale);
                            if (ImGui::Combo("Mode", &mode, "Follow\0Ragdoll\0Quasi static\0")) {
                                rtx.setCharacterMode(static_cast<fbik::Mode>(mode));
                                v.animationPlaying=rtx.animating();
                            }
                            ImGui::PopItemWidth();
                        }
                        if (hasCharacter && rtx.characterMode()==fbik::Mode::Follow)
                        {
                            if (ImGui::Button(v.animationPlaying ? "Pause animation" : "Play animation"))
                            {
                                v.animationPlaying = !v.animationPlaying;
                                if (v.animationPlaying)
                                {
                                    preparePlayback(v);
                                    v.playing = true;
                                    previousTime = glfwGetTime();
                                    accumulator = 0;
                                }
                                else rtx.stopAnimation();
                            }
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("Start walking, or hold the current pose while simulation continues.");
                            sameLineIfFits("Loop", true);
                            if (ImGui::Checkbox("Loop", &v.loop)) rtx.setAnimationLoop(v.loop);
                            float time = float(rtx.animationTime());
                            ImGui::PushItemWidth(210 * 0.7f * scale);
                            const bool seek = ImGui::SliderFloat("Time", &time, float(rtx.animationStart()),
                                                               float(rtx.animationEnd()), "%.2f s");
                            ImGui::PopItemWidth();
                            if (seek)
                            {
                                v.animationPlaying = false;
                                rtx.stopAnimation();
                                rtx.seekAnimation(time);
                            }
                        }
                        ImGui::PushItemWidth(210 * 0.7f * scale);
                        ImGui::SliderFloat("Speed", &v.playbackSpeed, 0.1f, 2.0f, "%.2fx");
                        ImGui::PopItemWidth();
                        if (hasCharacter)
                        {
                            bool show = rtx.boneCollidersVisible();
                            if (ImGui::Checkbox("Show bone colliders", &show)) rtx.showBoneColliders(show);
                        }
                        if (ImGui::Button("Frame stage")) frameScene(v);
                        if (ImGui::Button("Frame character") && hasCharacter) frameCharacter(v);
                        if (!hasCharacter && ImGui::IsItemHovered())
                            ImGui::SetTooltip("The current scene has no character.");
                        sameLineIfFits("Follow camera", true);
                        bool follow = hasCharacter && v.follow;
                        if (ImGui::Checkbox("Follow camera", &follow) && hasCharacter) v.follow = follow;
                        if (!hasCharacter && ImGui::IsItemHovered())
                            ImGui::SetTooltip("The current scene has no character.");
                        if (ImGui::Button(recorder.active() ? "Stop recording" : "Record movie"))
                        {
                            if (recorder.active()) stopRecording();
                            else if (!renderFailed)
                            {
                                const auto path = selectMovieFile(window);
                                if (!path.empty())
                                {
                                    recorder.start(path, width, height, 30, false);
                                    recordingStatus.clear();
                                    error.clear();
                                }
                            }
                            previousTime = glfwGetTime();
                            accumulator = 0;
                        }
                    }
                    catch (const std::exception &e)
                    {
                        error = e.what();
                        v.playing = false;
                    }
                }
                if (hasCharacter)
                {
                    ImGui::Separator();
                    if (ImGui::CollapsingHeader("Info"))
                    {
                        ImGui::Text("Character: %zu joints", rtx.characterJointCount());
                        ImGui::Text("Mapped: %zu bodies", rtx.boneColliderCount());
                        ImGui::Text("Connected: %zu joints", rtx.boneJointCount());
                        ImGui::TextWrapped(rtx.characterMode()==fbik::Mode::Follow
                            ? "Kinematic bone bodies follow the animation."
                            : rtx.characterMode()==fbik::Mode::QuasiStatic
                            ? "Gravity is off and momentum is cleared each step. Contacts and joint limits remain active."
                            : "Dynamic bone bodies drive the character. Animation is held.");
                    }
                }
            }
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Camera controls"))
                ImGui::TextWrapped("Shift + left drag: grab object and resume\nLeft drag: orbit\nRight drag: "
                                       "look\nMiddle drag: pan\nWheel: move\n"
                                       "W/A/S/D, Q/E: move\nSpace: play/pause\nHome: frame | Escape: close");
            ImGui::TextUnformatted("Drop a USD file to load it.");
            if (!v.pendingLoad.empty())
                ImGui::TextWrapped("Loading USD... First RTX initialization may take several minutes.");
            if (!error.empty())
            {
                ImGui::Separator();
                ImGui::TextWrapped("%s", error.c_str());
            }
            if (recordingSavedPopupPending)
            {
                ImGui::OpenPopup("Movie saved");
                recordingSavedPopupUntil = glfwGetTime() + 5;
                recordingSavedPopupPending = false;
            }
            if (ImGui::BeginPopup("Movie saved"))
            {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ui(360));
                ImGui::TextWrapped("%s", recordingStatus.c_str());
                ImGui::PopTextWrapPos();
                if (ImGui::Button("OK") || glfwGetTime() >= recordingSavedPopupUntil)
                    ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            ImGui::End();
            ImGui::Render();
            ImGui_ImplGLFW_RenderDrawData(ImGui::GetDrawData());
            if (frameLimit && v.pendingLoad.empty() && ++frames >= frameLimit)
            {
                if (!error.empty()) throw std::runtime_error(error);
                if (!screenshotPath.empty())
                {
                    std::vector<uint8_t> rgb(size_t(width) * height * 3);
                    glPixelStorei(GL_PACK_ALIGNMENT, 1);
                    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
                    std::ofstream output(screenshotPath, std::ios::binary);
                    output << "P6\n" << width << ' ' << height << "\n255\n";
                    for (int y = height - 1; y >= 0; --y)
                        output.write(reinterpret_cast<const char *>(rgb.data() + size_t(y) * width * 3), width * 3);
                    if (!output) throw std::runtime_error("Could not write the GUI screenshot.");
                }
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            if (recorder.active())
            {
                try
                {
                    if (renderFailed) stopRecording();
                    else recorder.write(rtx.pixels().data(), rtx.pixels().size());
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    try { recorder.stop(); }
                    catch (const std::exception &stop) { error += std::string("\n") + stop.what(); }
                }
            }
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
                    stopRecording();
                    rtx.initialize(executableDirectory.u8string());
                    rtx.load(requested, width, height);
                    renderFailed = false;
                    v.playing = false;
                    v.animationPlaying = false;
                    rtx.setAnimationLoop(v.loop);
                    followCenter = rtx.characterCenter();
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
                    if (!frameLimit)
                    {
                        recent.add(requested);
                        recent.save();
                    }
                    std::printf("Loaded USD: %s\n", requested.c_str());
                }
                catch (const std::exception &e)
                {
                    error = e.what();
                    std::fprintf(stderr, "%s\n", e.what());
                }
                const auto title = rtx.loaded() ? "FBIK PhysX - " + std::filesystem::u8path(rtx.path()).make_preferred().u8string()
                                                : "FBIK PhysX - USD Viewer";
                glfwSetWindowTitle(window, title.c_str());
                previousTime = glfwGetTime();
                accumulator = 0;
            }
        }
        stopRecording();
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n", e.what());
        exitCode = 1;
    }
    v.rtx = nullptr;
    glDeleteTextures(1, &texture);
    ImGui_ImplGLFW_Shutdown();
    ImGui::DestroyContext(context);
    glfwDestroyWindow(window);
    glfwTerminate();
    return exitCode;
}
