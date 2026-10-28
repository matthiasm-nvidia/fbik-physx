#include "PhysicsSimulation.h"
#include "Camera.h"
#include <ovstage/ovstage_config.h>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
int main(int argc, char **argv)
{
    ovstage_instance_t *stage = nullptr;
    bool initialized = false;
    int result = 0;
    try
    {
        require(argc == 2, "Expected project root");
        viewer::Camera camera;
        camera.init();
        camera.look_at({0, 0, 6}, {0, 0, 0});
        const auto centerRay = camera.screenRay(400, 300, 800, 600);
        require((centerRay.direction - camera.forward).magnitude() < 1e-6f, "Perspective center ray mismatch");
        const auto corner = camera.screenRay(800, 0, 800, 600);
        require(corner.direction.dot(camera.right) > 0 && corner.direction.dot(camera.up) > 0, "Picking axes inverted");
        camera.orthographic = true;
        camera.orthographicHeight = 6;
        const auto ortho = camera.screenRay(800, 0, 800, 600);
        require((ortho.direction - camera.forward).magnitude() < 1e-6f &&
                std::abs((ortho.origin-camera.pos).dot(camera.right)-4) < 1e-6f &&
                std::abs((ortho.origin-camera.pos).dot(camera.up)-3) < 1e-6f, "Orthographic picking ray mismatch");
        const auto root = std::filesystem::absolute(argv[1]);
        const auto binaries = (root / "deps/ovstage/bin").u8string();
        auto entry = ovstage_config_entry_binary_package_root_path(viewer::stage::str(binaries.c_str()));
        ovstage_config_t config{&entry, 1};
        viewer::stage::check(ovstage_initialize(&config));
        initialized = true;
        viewer::PhysicsRuntime physics;
        ovstage_instance_desc_t desc{};
        desc.name = "falling-box-test";
        viewer::stage::check(ovstage_create_instance(&desc, &stage));
        const auto path = (root / "resources/scenes/falling_box.usda").u8string();
        auto op = ovstage_population_open_usd_from_file(stage, viewer::stage::str(path.c_str()), 1, 0,
                                                       OVSTAGE_POPULATION_DOMAIN_ALL);
        viewer::stage::check(op.status);
        ovstage_population_op_wait_result_t wait{};
        viewer::stage::check(ovstage_population_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait));
        require(wait.error_op_id_count == 0, "USD population failed");
        ovstage_ordinal_t ordinal = 1;
        viewer::stage::seal(stage, ordinal);
        viewer::stage::hierarchy(stage, ordinal);
        const auto pose = [&] {
            return viewer::stage::read(stage, ordinal, {"omni:xform", "omni:fabric:worldMatrix"});
        };
        const auto initial = pose();
        require(std::abs(initial.at("/World/Box").at("omni:fabric:worldMatrix").numbers[13] - 1) < 1e-8, "Initial height");
        {
            viewer::PhysicsSimulation simulation(stage, ordinal);
            require(simulation.beginDrag({0, 1, 6}, {0, 0, -1}, ordinal), "Could not pick before the first physics step");
            require(pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers ==
                    initial.at("/World/Box").at("omni:fabric:worldMatrix").numbers, "Picking advanced the visible pose");
            simulation.stop(ordinal);
        }
        std::vector<double> trajectory;
        for (int run = 0; run < 2; ++run)
        {
            viewer::PhysicsSimulation simulation(stage, ordinal);
            for (int i = 0; i < 180; ++i)
            {
                simulation.step(ordinal);
                const auto state = pose();
                const auto height = state.at("/World/Box").at("omni:fabric:worldMatrix").numbers[13];
                if (!run) trajectory.push_back(height);
                else require(std::abs(height - trajectory[i]) < 1e-5, "Restart trajectory differs: stale velocity/contact state");
                require(std::abs(state.at("/World/Box/Geometry").at("omni:fabric:worldMatrix").numbers[13] - height) < 1e-8,
                        "Child geometry did not follow the rigid body");
                require(state.at("/World/Ground").at("omni:fabric:worldMatrix").numbers ==
                        initial.at("/World/Ground").at("omni:fabric:worldMatrix").numbers, "Static ground moved");
                if (i == 20) require(height < 0.5 && height > 0.2, "Box did not fall under gravity");
                if (i == 179)
                {
                    std::cout << "Resting height: " << height << '\n';
                    require(std::abs(height - 0.05) < 0.01, "Box did not settle on ground");
                }
            }
            simulation.stop(ordinal);
            const auto restored = pose();
            for (const auto &[path, prim] : initial)
                for (const auto &[name, value] : prim)
                    require(restored.at(path).at(name).numbers == value.numbers, "Stop did not restore original transform");
        }
        {
            viewer::PhysicsSimulation simulation(stage, ordinal);
            for (int i = 0; i < 180; ++i) {
                simulation.step(ordinal, false);
                if (i % 6 == 5) {
                    simulation.publish(ordinal);
                    require(std::abs(pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers[13]-trajectory[i])<1e-5,
                            "Deferred publication changed the simulated trajectory");
                    const auto published=ordinal;
                    simulation.publish(ordinal);
                    require(ordinal==published,"Publishing unchanged physics rewrote the stage");
                }
            }
            require(!simulation.beginDrag({0.3f, 3, 0}, {0, -1, 0}, ordinal), "Static ground was draggable");
            require(!simulation.beginDrag({20, 3, 0}, {0, 1, 0}, ordinal), "Empty ray picked a body");
            require(simulation.beginDrag({0, 0.05f, 6}, {0, 0, -1}, ordinal), "Could not pick resting box");
            simulation.moveDrag({0, -3, 6}, {0, 0, -1}, ordinal);
            for (int i = 0; i < 60; ++i) simulation.step(ordinal);
            require(pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers[13] > 0.02,
                    "Dragging bypassed ground collision");
            simulation.moveDrag({0, 3.5f, 6}, {0, 0, -1}, ordinal);
            for (int i = 0; i < 180; ++i) simulation.step(ordinal);
            const auto raised = pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers;
            std::cout << "Dragged height: " << raised[13] << '\n';
            require(raised[13] > 2, "Mouse joint did not lift the box");
            simulation.endDrag(ordinal);
            require(!simulation.dragging(), "Release did not end drag");
            for (int i = 0; i < 180; ++i) simulation.step(ordinal);
            require(std::abs(pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers[13] - 0.05) < 0.03,
                    "Released box did not settle on ground");
            const auto resting = pose().at("/World/Box").at("omni:fabric:worldMatrix").numbers;
            require(simulation.beginDrag({float(resting[12]), float(resting[13]), 6}, {0, 0, -1}, ordinal), "Second grab failed");
            simulation.stop(ordinal);
            const auto prims = viewer::stage::read(stage, ordinal, {"usd-prim-type"});
            require(!prims.count("/__FbikMouseJoint"), "Stop retained temporary USD joint");
            require(!simulation.dragging(), "Stop retained mouse joint");
            require(pose().at("/World/Box").at("omni:xform").numbers == initial.at("/World/Box").at("omni:xform").numbers,
                    "Stop after dragging did not restore pose");
        }
        viewer::stage::check(ovstage_destroy_instance(stage));
        stage = nullptr;
        std::cout << "Gravity, contact, child transforms, exact reset and repeatability passed.\n";
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; result = 1; }
    if (stage) ovstage_destroy_instance(stage);
    if (initialized) ovstage_shutdown();
    return result;
}
