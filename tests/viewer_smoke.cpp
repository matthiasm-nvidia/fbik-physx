#include "RtxView.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "viewer_imgui_theme.h"
#include <GLFW/glfw3.h>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

void require(bool value, const char *message)
{
    if (!value)
        throw std::runtime_error(message);
}
int main(int argc, char **argv)
{
    GLFWwindow *window = nullptr;
    ImGuiContext *context = nullptr;
    int result = 0;
    try
    {
        require(argc == 2, "Expected project root");
        const auto root = std::filesystem::absolute(argv[1]);
        require(glfwInit() != 0, "GLFW initialization failed");
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
        window = glfwCreateWindow(640, 480, "Viewer smoke test", nullptr, nullptr);
        require(window != nullptr, "OpenGL window creation failed");
        glfwMakeContextCurrent(window);
        context = ImGui::CreateContext();
        ImGui::GetIO().IniFilename = nullptr;
        viewer::imgui_theme::init_ui_scale();
        viewer::imgui_theme::apply_kit_dark_style();
        require(viewer::imgui_theme::load_ui_font(window), "Bundled UI font missing");
        ImGui_ImplGLFW_Init(window, false);
        {
            viewer::RtxView rtx;
            rtx.initialize(std::filesystem::absolute(argv[0]).parent_path().u8string());
            const auto file = (root / "tests/data/viewer.usda").u8string();
            rtx.load(file, 640, 480);
            require(rtx.loaded() && rtx.radius() > 1, "Stage failed to load or frame");
            viewer::Camera camera;
            camera.init();
            camera.look_at(viewer::Vec3(5, 3, 5), rtx.center());
            for (int i = 0; i < 4; ++i)
                rtx.render(camera, 640, 480);
            require(rtx.width() == 640 && rtx.height() == 480, "Initial image size mismatch");
            const auto &pixels = rtx.pixels();
            size_t varied = 0;
            for (size_t i = 4; i < pixels.size(); i += 4)
                varied += pixels[i] != pixels[0] || pixels[i+1] != pixels[1] || pixels[i+2] != pixels[2];
            require(varied > 1000, "Rendered image is uniform");
            std::ofstream image(root / "build/viewer-smoke.ppm", std::ios::binary);
            image << "P6\n640 480\n255\n";
            for (size_t i = 0; i < pixels.size(); i += 4)
                image.write(reinterpret_cast<const char *>(pixels.data() + i), 3);
            image.close();
            rtx.render(camera, 800, 450);
            require(rtx.width() == 800 && rtx.height() == 450, "Resize did not reach RTX");
            bool failed = false;
            try { rtx.load((root / "build/missing.usda").u8string(), 800, 450); }
            catch (const std::exception &) { failed = true; }
            require(failed && rtx.loaded() && rtx.path() == file, "Missing-file load lost previous stage");
            const auto invalid = root / "build/invalid.usda";
            { std::ofstream bad(invalid); bad << "This is not a USD stage.\n"; }
            failed = false;
            try { rtx.load(invalid.u8string(), 800, 450); }
            catch (const std::exception &) { failed = true; }
            require(failed && rtx.loaded() && rtx.path() == file, "Malformed-file load lost previous stage");
            rtx.render(camera, 800, 450);
            rtx.load(file, 640, 480);
            rtx.render(camera, 640, 480);
            rtx.load((root / "resources/scenes/falling_box.usda").u8string(), 640, 480);
            camera.look_at(viewer::Vec3(9, 7, 10), viewer::Vec3(0, 1.5f, 0));
            const auto boxHeightInImage = [&] {
                for (int i = 0; i < 4; ++i) rtx.render(camera, 640, 480);
                double sum = 0, count = 0;
                const auto &rgba = rtx.pixels();
                for (int y = 0; y < 480; ++y)
                    for (int x = 0; x < 640; ++x)
                    {
                        const size_t pixel = (y * 640 + x) * 4;
                        if (rgba[pixel] > 80 && rgba[pixel] > 1.35 * rgba[pixel+1] && rgba[pixel] > 1.35 * rgba[pixel+2])
                        { sum += y; ++count; }
                    }
                require(count > 50, "Falling box is not visible in RTX image");
                std::cout << "Box image centroid: " << sum/count << " pixels=" << count << std::endl;
                return sum / count;
            };
            const auto startY = boxHeightInImage();
            require(!rtx.beginDrag({{0, 4, 6}, {0, 0, -1}}), "Picking was enabled while stopped");
            rtx.startSimulation();
            require(rtx.simulating(), "Start did not enter simulation");
            for (int i = 0; i < 180; ++i)
            {
                rtx.stepSimulation();
                if (i % 10 == 0) rtx.render(camera, 640, 480);
            }
            const auto endY = boxHeightInImage();
            require(std::abs(endY - startY) > 30, "RTX did not show the falling box motion");
            camera.pos.x += 0.01f; // Camera hierarchy updates must preserve the simulated pose.
            require(std::abs(boxHeightInImage() - endY) < 3, "Camera movement reset simulated geometry");
            require(rtx.beginDrag({{0, 0.5f, 6}, {0, 0, -1}}), "RTX stage box could not be picked");
            rtx.moveDrag({{0, 3.5f, 6}, {0, 0, -1}});
            for (int i = 0; i < 120; ++i)
            {
                rtx.stepSimulation();
                if (i % 10 == 0) rtx.render(camera, 640, 480);
            }
            require(std::abs(boxHeightInImage() - endY) > 30, "RTX did not display mouse-joint motion");
            rtx.endDrag();
            require(!rtx.dragging(), "Mouse release retained joint");
            for (int i = 0; i < 180; ++i) rtx.stepSimulation();
            require(std::abs(boxHeightInImage() - endY) < 15, "Released box did not return to floor");
            rtx.stopSimulation();
            require(!rtx.simulating(), "Stop did not leave simulation");
            require(std::abs(boxHeightInImage() - startY) < 3, "RTX did not show restored pose");
            const auto boxX = [&] {
                boxHeightInImage();
                double sum=0,count=0;
                const auto &rgba=rtx.pixels();
                for (int y=0;y<480;++y) for (int x=0;x<640;++x) {
                    const auto i=(y*640+x)*4;
                    if (rgba[i]>80 && rgba[i]>1.35*rgba[i+1] && rgba[i]>1.35*rgba[i+2]) {sum+=x;++count;}
                }
                require(count>50,"Animated test cube is not visible");
                return sum/count;
            };
            const auto saveFrame = [&](const char *name) {
                std::ofstream out(root/"build"/name,std::ios::binary);
                out << "P6\n640 480\n255\n";
                const auto &rgba=rtx.pixels();
                for(size_t i=0;i<rgba.size();i+=4) out.write(reinterpret_cast<const char*>(rgba.data()+i),3);
            };
            rtx.startAnimation();
            require(rtx.animating() && rtx.simulating(),"Animation did not start physics");
            for(int i=0;i<75;++i) rtx.stepSimulation();
            const double beforePush=boxX();saveFrame("walk-before.ppm");
            for(int i=75;i<420;++i) {
                rtx.stepSimulation();
                if(i%20==0) rtx.render(camera,640,480);
                if(i==180) saveFrame("walk-middle.ppm");
            }
            const double afterPush=boxX();saveFrame("walk-after.ppm");
            require(std::abs(afterPush-beforePush)>25,"RTX did not show the character pushing the cube");
            rtx.stopAnimation();
            require(!rtx.animating() && !rtx.simulating(),"Stop animation did not stop physics");
            require(std::abs(boxHeightInImage()-startY)<3,"Animation stop did not visibly reset the cube");
            rtx.startAnimation();rtx.stepSimulation();
            rtx.stopSimulation();
            require(!rtx.animating(),"Stopping physics left animation active");
            rtx.startSimulation();
            rtx.stepSimulation();
            require(rtx.beginDrag({{0, 4, 6}, {0, 0, -1}}), "Grab before scene replacement failed");
            rtx.load(file, 640, 480);
            require(!rtx.dragging(), "Scene replacement retained drag state");
            require(!rtx.simulating(), "Loading a new stage retained the old simulation");
            glViewport(0, 0, 640, 480);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplGLFW_NewFrame();
            ImGui::Begin("USD Viewer");
            ImGui::Button("Load USD...");
            ImGui::End();
            ImGui::Render();
            ImGui_ImplGLFW_RenderDrawData(ImGui::GetDrawData());
            require(glGetError() == GL_NO_ERROR, "ImGui/OpenGL rendering failed");
            std::cout << "RTX rendering, framing, resize, reload, failed-load recovery and ImGui passed.\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    if (context)
    {
        ImGui_ImplGLFW_Shutdown();
        ImGui::DestroyContext(context);
    }
    if (window)
        glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
