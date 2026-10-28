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
    {
        std::cerr << "Assertion failed: " << message << std::endl;
        throw std::runtime_error(message);
    }
}
int main(int argc, char **argv)
{
    GLFWwindow *window = nullptr;
    ImGuiContext *context = nullptr;
    int result = 0;
    try
    {
        std::cout << std::unitbuf;
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
            require(!rtx.beginDrag(camera.screenRay(320, 240, 640, 480)) && !rtx.simulating(),
                    "Picking a stage without rigid bodies started physics");
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
            rtx.load((root / "resources/scenes/basic_walk.usda").u8string(), 640, 480);
            require(rtx.characterJointCount() > 0 && rtx.characterRadius() > 0,
                    "Character framing data is missing");
            require(std::abs(rtx.animationStart()) < 1e-6 && std::abs(rtx.animationEnd() - 8) < 1e-6,
                    "Timeline did not read the USD time range in seconds");
            const auto initialCharacter = rtx.characterCenter();
            rtx.seekAnimation(4);
            require(!rtx.simulating() && std::abs(rtx.animationTime() - 4) < 1e-6,
                    "Scrubbing started physics or failed to update time");
            require((rtx.characterCenter() - initialCharacter).magnitude() > 1,
                    "Scrubbing did not update character framing/follow position");
            rtx.showBoneColliders(true);
            rtx.render(camera, 640, 480);
            require(std::abs(rtx.animationTime() - 4) < 1e-6, "Rendering advanced a paused timeline");
            rtx.stopSimulation();
            require(std::abs(rtx.animationTime()) < 1e-6 &&
                    (rtx.characterCenter() - initialCharacter).magnitude() < 1e-4,
                    "Reset did not restore a scrubbed character without physics");
            rtx.showBoneColliders(false);
            require(rtx.initialCamera() != nullptr, "Basic walk camera is missing");
            camera = *rtx.initialCamera();
            viewer::Vec3 boxPixel{0, 0, 0};
            const auto boxHeightInImage = [&] {
                for (int i = 0; i < 4; ++i) rtx.render(camera, 640, 480);
                double sum = 0, sumX = 0, count = 0;
                const auto &rgba = rtx.pixels();
                for (int y = 0; y < 480; ++y)
                    for (int x = 0; x < 640; ++x)
                    {
                        const size_t pixel = (y * 640 + x) * 4;
                        if (rgba[pixel] > 80 && rgba[pixel] > 1.35 * rgba[pixel+1] && rgba[pixel] > 1.35 * rgba[pixel+2])
                        { sum += y; sumX += x; ++count; }
                    }
                require(count > 50, "Falling box is not visible in RTX image");
                boxPixel = {float(sumX/count + 0.5), float(sum/count + 0.5), 0};
                std::cout << "Box image centroid: " << sum/count << " pixels=" << count << std::endl;
                return sum / count;
            };
            const auto startY = boxHeightInImage();
            require(!rtx.beginDrag({{20, 3, 0}, {0, 1, 0}}) && !rtx.simulating(),
                    "Missing a body started simulation");
            require(rtx.beginDrag(camera.screenRay(boxPixel.x, boxPixel.y, 640, 480)),
                    "Could not pick the rendered box before starting simulation");
            require(rtx.simulating() && !rtx.animating() && std::abs(rtx.animationTime()) < 1e-6,
                    "Picking started animation");
            require(std::abs(boxHeightInImage() - startY) < 3, "Picking moved the initial box");
            rtx.endDrag();
            rtx.stopSimulation();
            rtx.startSimulation();
            require(rtx.simulating(), "Start did not enter simulation");
            for (int i = 0; i < 180; ++i)
            {
                rtx.stepSimulation();
                if (i % 10 == 0) rtx.render(camera, 640, 480);
            }
            const auto endY = boxHeightInImage();
            require(!rtx.animating() && std::abs(rtx.animationTime()) < 1e-6 &&
                    (rtx.characterCenter() - initialCharacter).magnitude() < 1e-4,
                    "Starting physics also started animation");
            require(std::abs(endY - startY) > 30, "RTX did not show the falling box motion");
            camera.pos.x += 0.01f; // Camera hierarchy updates must preserve the simulated pose.
            require(std::abs(boxHeightInImage() - endY) < 3, "Camera movement reset simulated geometry");
            require(rtx.beginDrag(camera.screenRay(boxPixel.x, boxPixel.y, 640, 480)),
                    "Rendered resting box could not be picked");
            rtx.moveDrag(camera.screenRay(boxPixel.x, boxPixel.y - 100, 640, 480));
            for (int i = 0; i < 120; ++i)
            {
                rtx.stepSimulation();
                if (i % 10 == 0) rtx.render(camera, 640, 480);
            }
            const auto liftedY = boxHeightInImage();
            require(endY - liftedY > 30, "RTX did not display mouse-joint motion");
            rtx.endDrag();
            require(!rtx.dragging(), "Mouse release retained joint");
            for (int i = 0; i < 180; ++i) rtx.stepSimulation();
            require(boxHeightInImage() - liftedY > 30, "Released box did not fall after dragging");
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
            require(!rtx.animating() && !rtx.simulating(),"Animation started physics while stopped");
            rtx.startSimulation();
            rtx.startAnimation();
            require(rtx.animating() && rtx.simulating(),"Animation did not start during simulation");
            for(int i=0;i<20;++i) rtx.stepSimulation();
            const double fallingBeforePause=boxHeightInImage();
            rtx.stopAnimation();
            require(!rtx.animating() && rtx.simulating(),"Stopping animation stopped physics");
            for(int i=0;i<30;++i) rtx.stepSimulation();
            require(std::abs(boxHeightInImage()-fallingBeforePause)>10,"Physics did not continue while animation was stopped");
            rtx.startAnimation();
            for(int i=20;i<75;++i) rtx.stepSimulation();
            const double beforePush=boxX();saveFrame("walk-before.ppm");
            for(int i=75;i<420;++i) {
                rtx.stepSimulation();
                if(i%20==0) rtx.render(camera,640,480);
                if(i==180) saveFrame("walk-middle.ppm");
            }
            const double afterPush=boxX();saveFrame("walk-after.ppm");
            require(std::abs(afterPush-beforePush)>25,"RTX did not show the character pushing the cube");
            rtx.stopAnimation();
            require(!rtx.animating() && rtx.simulating(),"Stop animation did not leave physics running");
            require(std::abs(boxX()-afterPush)<3,"Stop animation reset the cube");
            rtx.startAnimation();rtx.stepSimulation();
            require(std::abs(boxX()-afterPush)<3,"Resuming animation reset the scene");
            rtx.stopSimulation();
            require(std::abs(boxHeightInImage()-startY)<3,"Stop simulation did not reset the scene");
            require(!rtx.animating(),"Stopping physics left animation active");
            std::cout << "Checking timeline loop and final-pose hold.\n" << std::flush;
            rtx.startSimulation();
            rtx.setAnimationLoop(true);
            rtx.seekAnimation(rtx.animationEnd() - 0.005);
            rtx.startAnimation();
            rtx.stepSimulation();
            require(rtx.animating() && rtx.animationTime() < 0.02, "Loop did not wrap to the clip start");
            rtx.setAnimationLoop(false);
            rtx.seekAnimation(rtx.animationEnd() - 0.005);
            rtx.stepSimulation();
            require(!rtx.animating() && std::abs(rtx.animationTime() - rtx.animationEnd()) < 1e-6,
                    "Non-looping playback did not hold its final pose");
            rtx.stopSimulation();
            std::cout << "Timeline loop and final-pose hold passed.\n" << std::flush;
            rtx.seekAnimation(.5);
            const auto releaseCenter=rtx.characterCenter();
            rtx.setCharacterMode(fbik::Mode::Ragdoll);
            require(!rtx.simulating() && rtx.boneJointCount()==60,"Selecting ragdoll started simulation or lost joints");
            require((rtx.characterCenter()-releaseCenter).magnitude()<.001,"Selecting ragdoll changed the held pose");
            rtx.startSimulation();
            rtx.startAnimation();
            rtx.seekAnimation(4);
            require(!rtx.animating() && std::abs(rtx.animationTime()-.5)<1e-6,"Animation changed the ragdoll pose/time");
            for(int i=0;i<180;++i) {
                rtx.stepSimulation();
                if(i%20==0) rtx.render(camera,640,480);
            }
            require(rtx.characterCenter().y<releaseCenter.y-.3f,"Ragdoll framing did not follow the falling skeleton");
            rtx.render(camera,640,480);saveFrame("ragdoll.ppm");
            const auto heldCenter=rtx.characterCenter();
            camera.pos.x+=.01f;
            rtx.showBoneColliders(true);
            rtx.render(camera,640,480);
            require((rtx.characterCenter()-heldCenter).magnitude()<.001,"Rendering moved the paused ragdoll");
            rtx.setCharacterMode(fbik::Mode::Follow);
            require((rtx.characterCenter()-releaseCenter).magnitude()<.001,"Follow did not restore the held animation pose");
            rtx.stepSimulation();
            rtx.seekAnimation(.75); // Release a paused target that has not had a physics step yet.
            const auto scrubbedCenter=rtx.characterCenter();
            rtx.setCharacterMode(fbik::Mode::Ragdoll);
            rtx.stepSimulation();
            require((rtx.characterCenter()-scrubbedCenter).magnitude()<.03,"Ragdoll released an old physics pose after scrubbing");
            rtx.stopSimulation();
            require(rtx.characterMode()==fbik::Mode::Ragdoll && std::abs(rtx.animationTime())<1e-6 &&
                    (rtx.characterCenter()-initialCharacter).magnitude()<.001,"Reset did not restore the ragdoll start pose");
            rtx.startSimulation();rtx.stepSimulation();
            rtx.setCharacterMode(fbik::Mode::Follow);
            rtx.stopSimulation();rtx.showBoneColliders(false);
            std::cout << "Ragdoll playback, paused transitions, rendering and reset passed.\n" << std::flush;
            std::cout << "Restarting simulation before scene replacement.\n";
            rtx.startSimulation();
            std::cout << "Stepping before scene replacement.\n";
            rtx.stepSimulation();
            std::cout << "Picking before scene replacement.\n";
            require(rtx.beginDrag({{0, 4, 6}, {0, 0, -1}}), "Grab before scene replacement failed");
            std::cout << "Replacing scene.\n";
            rtx.load((root / "resources/scenes/falling_box.usda").u8string(), 640, 480);
            require(!rtx.dragging(), "Scene replacement retained drag state");
            require(!rtx.simulating(), "Loading a new stage retained the old simulation");
            require(rtx.characterJointCount() == 0 && rtx.boneColliderCount() == 0,
                    "Falling box retained a character after replacing Basic walk");
            rtx.render(camera, 640, 480);
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
