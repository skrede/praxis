#include "demo_machine.h"
#include "demo_machines.h"
#include "demo_documents.h"
#include "demo_configuration.h"
#include "scratch_documents.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/edited_list_window.h"
#include "praxis/manipulator/waypoints_configuration.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <filesystem>

// The documents ship beside the demonstration's sources and the descriptions they name are deployed
// beside its executable. A configure without the demonstration deploys neither, which is a valid
// configuration; the cases below then have nothing to compose and skip.
#ifndef PRAXIS_SHIPPED_MACHINE_DIR
    #define PRAXIS_SHIPPED_MACHINE_DIR ""
#endif
#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

using namespace praxis;
using namespace scratch_documents;

namespace {

// One registry over every document the demonstration ships, composed the way the application
// composes it: the shipped directory read from, a scratch directory written to, and the descriptions
// resolved where they were deployed.
struct offered
{
    explicit offered(const scratch &where, const std::filesystem::path &descriptions = PRAXIS_DEMO_RESOURCE_DIR)
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , registry(std::make_shared<scene::preset_registry>())
    {
        const config::outcome answered = config::load_or_defaults(
                demo::demonstration_keyspace(), config::resolve(std::filesystem::path(PRAXIS_SHIPPED_MACHINE_DIR) / "praxis-manipulator.xml", PRAXIS_SHIPPED_MACHINE_DIR));
        REQUIRE_FALSE(answered.failure.has_value());

        names = demo::register_arm_presets(registry, answered.values, demo::documents(PRAXIS_SHIPPED_MACHINE_DIR, where.state()), descriptions, nullptr);
    }

    // One composition, torn down the way the application tears one down, with whatever a case wants
    // out of it read off the preset while it still stands.
    template<typename Reading>
    auto read_composed(const std::string &named, Reading reading)
    {
        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};
        const scene::preset_site site{*scene, loop.main_strand(), *loop.make_strand(), [] {}, nowhere, nowhere, {}};

        std::shared_ptr<scene::preset> composed = registry->load_preset(named)(site);
        REQUIRE(composed != nullptr);

        auto answered = reading(*composed);

        composed->tear_down();
        REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
        REQUIRE(loop.drain().has_value());

        return answered;
    }

    // What the factory answers, which is nothing at all when the description the machine names could
    // not be read.
    std::shared_ptr<scene::preset> composed_or_nothing(const std::string &named)
    {
        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};
        const scene::preset_site site{*scene, loop.main_strand(), *loop.make_strand(), [] {}, nowhere, nowhere, {}};

        return registry->load_preset(named)(site);
    }

    std::vector<std::string> windows_of(const std::string &named)
    {
        return read_composed(named,
                             [](const scene::preset &composed)
                             {
                                 std::vector<std::string> titles;
                                 for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
                                     titles.push_back(panel->display_name());

                                 return titles;
                             });
    }

    std::vector<manipulator::joint_vector> waypoints_of(const std::string &named)
    {
        return read_composed(named,
                             [](const scene::preset &composed)
                             {
                                 std::vector<manipulator::joint_vector> rows;
                                 for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
                                     if(const auto listed = std::dynamic_pointer_cast<manipulator::joint_waypoint_list>(panel); listed != nullptr)
                                         rows = listed->state().rows;

                                 return rows;
                             });
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<scene::preset_registry> registry;
    std::vector<std::string> names;
};

bool descriptions_deployed()
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};

    return !root.empty() && std::filesystem::is_directory(root);
}

constexpr const char *point_to_point          = "kr6r: point to point motion";
constexpr const char *point_to_point_document = "kr6r-point-to-point.xml";

// Six joint values in radians, each different from the others and none of them a value the run this
// list opens at stands on, so a row read from the shipped document rather than the copy is plain.
manipulator::joint_vector typed_row()
{
    manipulator::joint_vector row(6);
    row << 0.11, 0.22, 0.33, 0.44, 0.55, 0.66;

    return row;
}

// A row saved the way the application saves one: into the copy the writing resolver reproduces from
// the shipped document, over the machine keyspace.
void save_joint_waypoint(const demo::documents &mine, const std::filesystem::path &named, const manipulator::joint_vector &row)
{
    const config::binding into{demo::machine_keyspace(), mine.writing(named), config::expectation::partial};
    const manipulator::joint_waypoint_list::settings typed{std::vector<manipulator::joint_vector>{row}};

    REQUIRE(config::save(into, manipulator::write_joint_waypoints(config::load_or_defaults(into).values, typed, presets::window_paths::joint_waypoints)).has_value());
}

}

// A spelling routed to its neighbour in the composer table would still register, still compose and
// still load; what it would open is the other preset's windows. Nothing else in the tree reads the
// order of that table, so this is where an entry standing one place over is caught.
TEST_CASE("each shipped document opens the windows its own scenario names", "[examples][registry]")
{
    if(!descriptions_deployed())
        SKIP("the demonstration's descriptions are not deployed in this configuration");

    const scratch where("shipped-scenarios");
    offered demonstration(where);

    REQUIRE(demonstration.names.size() == 12u);

    CHECK(demonstration.windows_of("kr6r: forward kinematics") == std::vector<std::string>{"Joint control", "Pose", "View"});
    CHECK(demonstration.windows_of("kr6r: screw chain preview") == std::vector<std::string>{"Joint control", "Chain", "View"});
    CHECK(demonstration.windows_of("kr6r: numerical inverse kinematics") ==
          std::vector<std::string>{"Joint control", "Target pose", "Starts", "Solutions", "Iterations", "Convergence", "View"});
    CHECK(demonstration.windows_of("kr6r: analytic inverse kinematics") == std::vector<std::string>{"Joint control", "Target pose", "Solutions", "View"});
    CHECK(demonstration.windows_of("ur3e: numerical inverse kinematics") ==
          std::vector<std::string>{"Joint control", "Target pose", "Starts", "Solutions", "Iterations", "Convergence", "View"});
    CHECK(demonstration.windows_of("kr6r: point to point motion") == std::vector<std::string>{"Joint control", "Waypoints", "Control parameters", "Preview"});
    CHECK(demonstration.windows_of("kr6r: via point motion") == std::vector<std::string>{"Joint control", "Waypoints", "Preview", "Joint curves"});
    CHECK(demonstration.windows_of("kr6r: path comparison") == std::vector<std::string>{"Joint control", "Comparison", "View"});
    CHECK(demonstration.windows_of("kr6r: velocity kinematics") == std::vector<std::string>{"Joint control", "Velocity kinematics", "Render controls", "View"});
}

// An order of events rather than a path: the presets are registered while no copy of any shipped
// document stands, a save then makes one, and the composition after it reads what the save left.
TEST_CASE("a preset saved into for the first time composes from the copy", "[examples][registry]")
{
    if(!descriptions_deployed())
        SKIP("the demonstration's descriptions are not deployed in this configuration");

    const scratch where("saving-before-composing");
    offered demonstration(where);
    REQUIRE_FALSE(std::filesystem::exists(where.state() / point_to_point_document));

    save_joint_waypoint(demo::documents(PRAXIS_SHIPPED_MACHINE_DIR, where.state()), point_to_point_document, typed_row());
    const std::vector<manipulator::joint_vector> opened = demonstration.waypoints_of(point_to_point);

    REQUIRE(opened.size() == 1u);
    REQUIRE(opened.front().isApprox(typed_row(), 1e-5));
}

// The names come from the shipped documents and the models from a directory beside the executable, so
// a copy of the application whose models never arrived still offers every name and can compose none
// of them. Composing one has to answer nothing rather than end the run.
TEST_CASE("a machine whose description is not there is refused by name rather than composed", "[examples][registry]")
{
    const scratch where("absent-descriptions");
    offered demonstration(where, where.state() / "descriptions-that-are-not-there");

    REQUIRE_FALSE(demonstration.names.empty());

    for(const std::string &named : demonstration.names)
        CHECK(demonstration.composed_or_nothing(named) == nullptr);

    SUCCEED("every name was refused and the run carried on to the end");
}
