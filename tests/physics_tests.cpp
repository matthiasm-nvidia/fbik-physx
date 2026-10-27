#include "PhysicsSimulation.h"
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
        require(std::abs(initial.at("/World/Box").at("omni:fabric:worldMatrix").numbers[13] - 4) < 1e-8, "Initial height");
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
                if (i == 20) require(height < 3.8 && height > 2, "Box did not fall under gravity");
                if (i == 179)
                {
                    std::cout << "Resting height: " << height << '\n';
                    require(std::abs(height - 0.5) < 0.03, "Box did not settle on ground");
                }
            }
            simulation.stop(ordinal);
            const auto restored = pose();
            for (const auto &[path, prim] : initial)
                for (const auto &[name, value] : prim)
                    require(restored.at(path).at(name).numbers == value.numbers, "Stop did not restore original transform");
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
