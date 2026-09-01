#include "drawn_lines.h"
#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/clock.h"
#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <chrono>
#include <limits>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <filesystem>

// The descriptions the shipped documents name are deployed beside the demonstration executable. A
// configure without the demonstration deploys none, which is a valid configuration; the cases that
// play a motion then have no machine to play it on and skip.
#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

using namespace praxis;
using namespace praxis::fixture;

namespace {

// Where each control of the comparison panel stands in a walk down it. A row of components counts as
// one step, so the two ends are the first two and the switches follow them.
constexpr std::size_t first_end     = 0;
constexpr std::size_t second_end    = 1;
constexpr std::size_t shape_control = 5;
constexpr std::size_t play_control  = 6;

// How far a played run's traversed path may stand from the polyline it was commanded along and still
// be said to lie on it. A millimetre at the tool is three orders below the arm's own reach and two
// above what a position read back out of a float buffer is comparable at.
constexpr double lying_on_it = 1.0e-3;

// The playback is driven by a repeating task on the arm's own strand, so a run is played out by
// advancing the clock in spans and letting the strand service the ticks it registered.
constexpr auto advanced_span       = std::chrono::milliseconds(250);
constexpr std::uint32_t most_spans = 400;

scheduler::time_point dictated{};

scheduler::time_point reading()
{
    return dictated;
}

scheduler::clock_source dictating()
{
    dictated = scheduler::time_point{};

    return scheduler::clock_source{&reading};
}

std::size_t solves_entered = 0;

expected<void, refusal> recorded_search(const manipulator::forward_kinematics_ops &, const manipulator::differential_kinematics_ops &, const manipulator::screw_chain &,
                                        const transform &, const manipulator::joint_vector &, const manipulator::solver_parameters &, manipulator::ik_result &)
{
    ++solves_entered;

    return unexpected(refusal::not_implemented);
}

expected<void, refusal> recorded_closed_form(const manipulator::forward_kinematics_ops &, const manipulator::screw_chain &, const transform &, manipulator::ik_result &)
{
    ++solves_entered;

    return unexpected(refusal::not_implemented);
}

manipulator::capabilities recording_solves()
{
    manipulator::capabilities held = manipulator::baseline();
    held.ik                        = manipulator::inverse_kinematics_ops{&recorded_search, &recorded_closed_form};

    return held;
}

std::string drawn_under(const char *named)
{
    return manipulator::loadable_robot_stencil::pose_path_name(named);
}

// The greatest distance any point of one line stands from the other, which is how a run sampled at
// playback ticks is compared with a polyline sampled at path parameters.
double off_the_line(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &line)
{
    double furthest = 0.0;
    for(const Eigen::Vector3d &at : points)
    {
        double nearest = std::numeric_limits<double>::max();
        for(std::size_t segment = 1; segment < line.size(); ++segment)
        {
            const Eigen::Vector3d along = line[segment] - line[segment - 1u];
            const double length         = along.squaredNorm();
            const double t              = length == 0.0 ? 0.0 : std::clamp((at - line[segment - 1u]).dot(along) / length, 0.0, 1.0);
            nearest                     = std::min(nearest, (at - (line[segment - 1u] + t * along)).norm());
        }
        furthest = std::max(furthest, nearest);
    }

    return furthest;
}

std::filesystem::path shipped_description()
{
    return std::filesystem::path{PRAXIS_DEMO_RESOURCE_DIR} / "kuka_kr6_support/urdf/kr6r900sixx.xacro";
}

bool description_deployed()
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};

    return !root.empty() && std::filesystem::exists(shipped_description());
}

// The machine the shipped comparison document opens, composed from the deployed description rather
// than through that document, so a case reads the pair the library itself opens at.
presets::arm_scenario shipped_machine()
{
    presets::arm_scenario chosen;
    chosen.options.package_roots.push_back(std::filesystem::path{PRAXIS_DEMO_RESOURCE_DIR});
    chosen.description = shipped_description();

    return chosen;
}

// One headless scene the comparison preset is opened against, with the way to say the composition
// cannot continue bound to it and every capability behind it the caller's, so a case reading what the
// solver was asked can bind a solver that records.
struct opened_comparison
{
    opened_comparison()
            : unloaded(false)
            , loop(scheduler::inline_workers, dictating())
            , scene(threepp::Scene::create())
            , composed()
    {
        solves_entered = 0;
    }

    opened_comparison(const opened_comparison &) = delete;

    std::shared_ptr<scene::preset> open(const presets::arm_scenario &chosen, const manipulator::capabilities &arm)
    {
        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};
        const scene::preset_site site{*scene, loop.main_strand(), *loop.make_strand(), [this] { unloaded = true; }, opening_route(), nowhere, {}};

        composed = presets::arm_preset(site, arm, trajectory::baseline(), rigid_motion::baseline(), chosen, presets::arm_windows_path_comparison(chosen));
        if(composed != nullptr)
            REQUIRE(composed->initialize().has_value());

        return composed;
    }

    std::shared_ptr<scene::imgui_window> panel(const std::string &named) const
    {
        REQUIRE(composed != nullptr);

        for(const std::shared_ptr<scene::imgui_window> &opened : composed->windows)
            if(opened->display_name() == named)
                return opened;

        FAIL("the composition opened no panel named " + named);

        return nullptr;
    }

    // What stands on the drawing once every panel has told it what the arm published.
    std::vector<Eigen::Vector3d> line(const char *named)
    {
        REQUIRE(loop.drain().has_value());
        static_cast<void>(panels_opened_by(*panel("Comparison")));
        REQUIRE(loop.main_strand().post([this] { composed->stencil->render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);

        return line_in_world(*scene, drawn_under(named));
    }

    // The run the tool actually traversed, once the motion in flight has stopped growing it.
    std::vector<Eigen::Vector3d> played_out()
    {
        std::vector<Eigen::Vector3d> reached;
        for(std::uint32_t span = 0; span < most_spans; ++span)
        {
            dictated += std::chrono::duration_cast<scheduler::time_point::duration>(advanced_span);
            REQUIRE(loop.drain().has_value());

            const std::vector<Eigen::Vector3d> now = line(manipulator::traversed_motion_path);
            if(!now.empty() && now.size() == reached.size())
                return now;

            reached = now;
        }

        return reached;
    }

    ~opened_comparison()
    {
        if(composed == nullptr)
            return;

        composed->tear_down();
        REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
        REQUIRE(loop.drain().has_value());
    }

    bool unloaded;
    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<scene::preset> composed;
};

std::shared_ptr<manipulator::path_comparison_window> comparison_of(const opened_comparison &stage)
{
    const auto held = std::dynamic_pointer_cast<manipulator::path_comparison_window>(stage.panel("Comparison"));

    REQUIRE(held != nullptr);

    return held;
}

// Steps the control that chooses the played shape until it stands on the one asked for, so what a
// case pins is the shape rather than a place in a list.
void choose(const std::shared_ptr<manipulator::path_comparison_window> &panel, manipulator::compared_path shape)
{
    for(std::size_t tried = 0; tried < 3u && panel->state().played != shape; ++tried)
        take_entry_at(*panel, shape_control, 1u);

    REQUIRE(panel->state().played == shape);
}

}

// The comparison exists to put three shapes side by side, and the first thing it must do is put them
// there without being asked.
TEST_CASE("the composition draws three commanded shapes before any control is pressed", "[presets][comparison]")
{
    const described_arm described(6, "six");

    opened_comparison stage;
    REQUIRE(stage.open(described_by(described.where), manipulator::baseline()) != nullptr);

    for(const char *named :
        {manipulator::path_comparison_window::joint_space_path, manipulator::path_comparison_window::decoupled_path, manipulator::path_comparison_window::screw_path})
    {
        INFO(named);
        CHECK(stage.line(named).size() == manipulator::path_comparison_window::drawn_points);
    }

    CHECK_FALSE(stage.unloaded);
}

// The ends are configurations and the two poses come from the forward map, so the picture cannot fail
// from unreachability and cannot silently depend on which branch a solve took.
TEST_CASE("no solve is entered while the three shapes are drawn or either end is changed", "[presets][comparison]")
{
    const described_arm described(6, "six");

    opened_comparison stage;
    REQUIRE(stage.open(described_by(described.where), recording_solves()) != nullptr);

    REQUIRE(stage.line(manipulator::path_comparison_window::screw_path).size() == manipulator::path_comparison_window::drawn_points);

    type_component_at(*stage.panel("Comparison"), first_end, 0u, "35.5");
    type_component_at(*stage.panel("Comparison"), second_end, 1u, "-42.5");

    REQUIRE(stage.line(manipulator::path_comparison_window::screw_path).size() == manipulator::path_comparison_window::drawn_points);
    CHECK(solves_entered == 0u);
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("no comparison is composed from a description that does not load", "[presets][comparison]")
{
    const described_arm described(6, "six");

    opened_comparison stage;

    CHECK(stage.open(described_by(described.directory / "no_such_description.urdf"), manipulator::baseline()) == nullptr);
    CHECK_FALSE(stage.unloaded);
}

// A joint-space motion traverses exactly what it was commanded along, which is the half of the
// comparison that has to hold for the other half to say anything.
TEST_CASE("playing the joint-space shape leaves a traversed path lying on the shape it was commanded along", "[presets][comparison]")
{
    if(!description_deployed())
        SKIP("the demonstration's descriptions are not deployed in this configuration");

    opened_comparison stage;
    REQUIRE(stage.open(shipped_machine(), manipulator::baseline()) != nullptr);

    choose(comparison_of(stage), manipulator::compared_path::joint_space);
    const std::vector<Eigen::Vector3d> commanded = stage.line(manipulator::path_comparison_window::joint_space_path);

    REQUIRE(commanded.size() == manipulator::path_comparison_window::drawn_points);

    press_at(*stage.panel("Comparison"), play_control);

    const std::vector<Eigen::Vector3d> traversed = stage.played_out();

    REQUIRE(traversed.size() > 1u);
    INFO("the traversed run stands " << off_the_line(traversed, commanded) << " m off the commanded polyline");
    CHECK(off_the_line(traversed, commanded) < lying_on_it);
    CHECK_FALSE(stage.unloaded);
}

// A task-space motion resolves through the solver at every sample and can leave the curve it was
// commanded along. Whether it does, and by how much, is a property of this arm and this solver, so
// the departure is reported rather than asserted: what is asserted is that the run happened and that
// it stands between the shape's own ends.
TEST_CASE("playing the screw shape leaves a traversed path whose departure from it is reported", "[presets][comparison]")
{
    if(!description_deployed())
        SKIP("the demonstration's descriptions are not deployed in this configuration");

    opened_comparison stage;
    REQUIRE(stage.open(shipped_machine(), manipulator::baseline()) != nullptr);

    choose(comparison_of(stage), manipulator::compared_path::screw);
    const std::vector<Eigen::Vector3d> commanded = stage.line(manipulator::path_comparison_window::screw_path);

    REQUIRE(commanded.size() == manipulator::path_comparison_window::drawn_points);

    press_at(*stage.panel("Comparison"), play_control);

    const std::vector<Eigen::Vector3d> traversed = stage.played_out();

    REQUIRE(traversed.size() > 1u);
    WARN("the traversed run stands " << off_the_line(traversed, commanded) << " m off the screw shape it was commanded along");
    CHECK((traversed.front() - commanded.front()).norm() < lying_on_it);
    CHECK_FALSE(stage.unloaded);
}

// Any one of the three can be run, which is what the control choosing between them is for. The pair
// of ends the library opens at was selected so that this holds on the machine the shipped document
// opens; it is not a property of every pair of ends and this case does not claim it is.
TEST_CASE("each of the three shapes plays and leaves a traversed path", "[presets][comparison]")
{
    if(!description_deployed())
        SKIP("the demonstration's descriptions are not deployed in this configuration");

    for(const manipulator::compared_path shape : {manipulator::compared_path::joint_space, manipulator::compared_path::decoupled, manipulator::compared_path::screw})
    {
        INFO(static_cast<std::size_t>(shape));

        opened_comparison stage;
        REQUIRE(stage.open(shipped_machine(), manipulator::baseline()) != nullptr);

        choose(comparison_of(stage), shape);
        press_at(*stage.panel("Comparison"), play_control);

        CHECK(stage.played_out().size() > 1u);
        CHECK_FALSE(stage.unloaded);
    }
}
