#include "drawn_lines.h"
#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/edited_list_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/plot_window.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The density one previewed motion is drawn at, written here so that changing it has to be done
// twice on purpose rather than once by accident.
constexpr std::size_t previewed_samples = 385;

// Where the panel's own controls stand in the walk down it: the ask, the play beside it, and the
// scrub below them once a preview stands.
constexpr std::size_t ask_control  = 0;
constexpr std::size_t play_control = 1;

// Where the parameters panel's controls stand: the playback rate, the scaling the motions are
// composed under, and the switch that overrides the trapezoid's bounds.
constexpr std::size_t scaling_control   = 1;
constexpr std::size_t trapezoid_control = 2;

// The last of the three scalings the control offers, which is neither the one a scenario opens at
// nor the one a list of choices stands on when it opens.
constexpr std::size_t another_scaling = 2;

const std::string &commanded_line()
{
    static const std::string named = manipulator::loadable_robot_stencil::pose_path_name(manipulator::commanded_motion_path);

    return named;
}

// One composer of a generated motion, named so a walk over the table says which of them failed.
struct trajectory_composer
{
    const char *named;
    manipulator::arm_composition (*compose)(presets::arm_scenario);
};

constexpr std::array trajectory_composers{trajectory_composer{"point to point", &presets::arm_windows_point_to_point},
                                          trajectory_composer{"via point", &presets::arm_windows_via_point},
                                          trajectory_composer{"path comparison", &presets::arm_windows_path_comparison}};

// One headless scene a motion scenario is opened against, with the way to say the composition cannot
// continue bound to it and the shipped implementation of every capability behind it, so what the
// controls below press is what the application presses.
struct opened_motion
{
    opened_motion()
            : unloaded(false)
            , loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , composed()
    {
    }

    opened_motion(const opened_motion &) = delete;

    std::shared_ptr<scene::preset> open(const presets::arm_scenario &chosen, const manipulator::arm_composition &opened)
    {
        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};
        const scene::preset_site site{*scene, loop.main_strand(), *loop.make_strand(), [this] { unloaded = true; }, opening_route(), nowhere, {}};

        composed = presets::arm_preset(site, manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline(), chosen, opened);
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

    // What the tool is drawn as having been commanded along, once the arm has answered whatever the
    // controls asked of it and the panel has told the drawing what it published.
    std::vector<Eigen::Vector3d> commanded()
    {
        REQUIRE(loop.drain().has_value());
        static_cast<void>(panels_opened_by(*panel("Preview")));
        REQUIRE(loop.main_strand().post([this] { composed->stencil->render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);

        return line_in_world(*scene, commanded_line());
    }

    ~opened_motion()
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

// What the preview panel plots, taken off the panel the composition opened rather than a second one
// built beside it.
scene::plot_reading previewed_by(const opened_motion &stage)
{
    const auto panel = std::dynamic_pointer_cast<manipulator::trajectory_preview_window>(stage.panel("Preview"));

    REQUIRE(panel != nullptr);

    return panel->reading();
}

std::size_t rows_of(const opened_motion &stage)
{
    const auto listed = std::dynamic_pointer_cast<manipulator::joint_waypoint_list>(stage.panel("Waypoints"));

    REQUIRE(listed != nullptr);

    return listed->state().rows.size();
}

}

// The rows run as separate motions between the waypoints, so the path drawn for them carries one
// motion's worth of samples per pair of rows rather than one motion's worth for the whole list.
TEST_CASE("asking for a preview draws the run of motions between the rows the list holds", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);
    CHECK(stage.commanded().empty());

    const std::size_t rows = rows_of(stage);

    REQUIRE(rows > 2u);

    press_at(*stage.panel("Preview"), ask_control);

    CHECK(stage.commanded().size() == (rows - 1u) * previewed_samples);
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("a row added after the ask leaves nothing standing that describes the rows as they were", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    const std::size_t rows = rows_of(stage);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    press_at(*stage.panel("Waypoints"), rows);

    CHECK(rows_of(stage) == rows + 1u);
    CHECK(stage.commanded().empty());
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("a row typed into after the ask leaves nothing standing that describes the rows as they were", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    type_component_at(*stage.panel("Waypoints"), 0u, 2u, "12.5");

    CHECK(stage.commanded().empty());
    CHECK_FALSE(stage.unloaded);
}

// The scaling is what the motions are composed under, so a preview taken before it changed describes
// a motion the arm would no longer play.
TEST_CASE("choosing another time scaling after the ask leaves nothing standing that describes the old one", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    take_entry_at(*stage.panel("Control parameters"), scaling_control, another_scaling);

    CHECK(stage.commanded().empty());
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("overriding the trapezoid bounds after the ask leaves nothing standing that describes the bounds it was taken under", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    press_at(*stage.panel("Control parameters"), trapezoid_control);

    CHECK(stage.commanded().empty());
    CHECK_FALSE(stage.unloaded);
}

// The arm's own movement is not an input the preview was computed from, which is what leaves the
// commanded path drawn for the tool to be watched against.
TEST_CASE("playing the run back leaves the commanded path standing", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);

    const std::vector<Eigen::Vector3d> drawn = stage.commanded();

    REQUIRE_FALSE(drawn.empty());

    press_at(*stage.panel("Preview"), play_control);

    CHECK(same_line(stage.commanded(), drawn));
    CHECK_FALSE(stage.unloaded);
}

// The rows run as one motion through every waypoint, so the path drawn for them carries one motion's
// worth of samples over the whole list rather than one per pair of rows -- which is the difference
// between this preset and the one beside it, read off the drawing itself.
TEST_CASE("asking for a preview draws one motion through every row the list holds", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_via_point(chosen)) != nullptr);
    CHECK(stage.commanded().empty());

    const std::size_t rows = rows_of(stage);

    REQUIRE(rows > 2u);

    press_at(*stage.panel("Preview"), ask_control);

    CHECK(stage.commanded().size() == previewed_samples);
    CHECK_FALSE(stage.unloaded);
}

// The polynomial the rows are fitted with carries its own timing, so a separately chosen scaling
// would have nothing to act on and no control for one is opened.
TEST_CASE("the via-point composition opens no control the arm's time scaling is chosen with", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    const std::shared_ptr<scene::preset> composed = stage.open(chosen, presets::arm_windows_via_point(chosen));

    REQUIRE(composed != nullptr);

    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        CHECK(panel->display_name() != "Control parameters");

    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("a row typed into after the via-point ask leaves nothing standing that describes the rows as they were", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_via_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    type_component_at(*stage.panel("Waypoints"), 0u, 2u, "12.5");

    CHECK(stage.commanded().empty());
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("playing the motion through the rows leaves the commanded path standing", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_via_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);

    const std::vector<Eigen::Vector3d> drawn = stage.commanded();

    REQUIRE_FALSE(drawn.empty());

    press_at(*stage.panel("Preview"), play_control);

    CHECK(same_line(stage.commanded(), drawn));
    CHECK_FALSE(stage.unloaded);
}

// A via-point spline has no set of scalings it could have been commanded under, so its panel draws
// the one curve the motion has where the run between the rows draws the three it was chosen from.
TEST_CASE("the via-point preview draws one curve per frame and the point-to-point preview three", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    {
        opened_motion stage;
        REQUIRE(stage.open(chosen, presets::arm_windows_via_point(chosen)) != nullptr);

        press_at(*stage.panel("Preview"), ask_control);
        REQUIRE_FALSE(stage.commanded().empty());

        const scene::plot_reading drawn = previewed_by(stage);

        CHECK(drawn.message.empty());
        REQUIRE(drawn.frames.size() == 3u);
        for(const scene::plot_frame &frame : drawn.frames)
        {
            CHECK(frame.series.size() == 1u);
            REQUIRE_FALSE(frame.series.empty());
            CHECK_FALSE(frame.series.front().ordinate.empty());
        }
        CHECK_FALSE(stage.unloaded);
    }

    opened_motion stage;
    REQUIRE(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) != nullptr);

    press_at(*stage.panel("Preview"), ask_control);
    REQUIRE_FALSE(stage.commanded().empty());

    const scene::plot_reading drawn = previewed_by(stage);

    CHECK(drawn.message.empty());
    REQUIRE(drawn.frames.size() == 3u);
    for(const scene::plot_frame &frame : drawn.frames)
    {
        CHECK(frame.series.size() == 3u);
        REQUIRE_FALSE(frame.series.empty());
        CHECK_FALSE(frame.series.front().ordinate.empty());
    }
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("no via-point arm is composed from a description that does not load", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.directory / "no_such_description.urdf");

    opened_motion stage;

    CHECK(stage.open(chosen, presets::arm_windows_via_point(chosen)) == nullptr);
    CHECK_FALSE(stage.unloaded);
}

TEST_CASE("no point-to-point arm is composed from a description that does not load", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.directory / "no_such_description.urdf");

    opened_motion stage;

    CHECK(stage.open(chosen, presets::arm_windows_point_to_point(chosen)) == nullptr);
    CHECK_FALSE(stage.unloaded);
}

// The tones the two drawn paths are separated at were selected against a population that leaves out
// the tone a joint told apart wears, on the premise that no composer of a generated motion reaches
// the control that tells one apart. That premise is enforced here.
TEST_CASE("no composer of a generated motion tells the drawing a joint apart", "[presets][trajectory]")
{
    for(const trajectory_composer &one : trajectory_composers)
    {
        INFO(one.named);

        const described_arm described(6, "six");
        const presets::arm_scenario chosen = described_by(described.where);

        opened_motion stage;
        const std::shared_ptr<scene::preset> composed = stage.open(chosen, one.compose(chosen));

        REQUIRE(composed != nullptr);

        for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
            static_cast<void>(panels_opened_by(*panel));

        CHECK_FALSE(dynamic_cast<manipulator::loadable_robot_stencil &>(*composed->stencil).selected_joint().has_value());
        CHECK_FALSE(stage.unloaded);
    }
}

// Nothing but a view control ever hides the arm's meshes, so a composition whose subject is the arm
// itself opens without one and the arm is drawn all the same.
TEST_CASE("a scenario of a generated motion opens no view window and draws the arm without one", "[presets][trajectory]")
{
    for(const trajectory_composer &one : {trajectory_composers[0], trajectory_composers[1]})
    {
        INFO(one.named);

        const described_arm described(6, "six");
        const presets::arm_scenario chosen = described_by(described.where);

        opened_motion stage;
        const std::shared_ptr<scene::preset> composed = stage.open(chosen, one.compose(chosen));

        REQUIRE(composed != nullptr);

        for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
            CHECK(panel->display_name() != "View");

        CHECK(drawn(rendered_arm(*stage.scene)));
    }
}

// The one control this scenario keeps is the one that takes the arm out of the way of the three
// drawn paths. The screw axes and the reach of them are not its subject, so neither is offered.
TEST_CASE("the path-comparison view offers the model control and nothing beside it", "[presets][trajectory]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_motion stage;

    REQUIRE(stage.open(chosen, presets::arm_windows_path_comparison(chosen)) != nullptr);

    scene::imgui_window &view = *stage.panel("View");

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    CHECK(navigable_items(frames, [&view] { view.render(); }) == 1u);
    CHECK(drawn(rendered_arm(*stage.scene)));
}
