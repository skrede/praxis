#include "window_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/control_parameters_window.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui_internal.h>

#include <threepp/scenes/Scene.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <functional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr const char *panel_title = "Preview";

// The scrub's own label, and a label no control in the panel carries: the path parameter is plotted
// rather than driven.
constexpr const char *scrub_control          = "Sample";
constexpr const char *path_parameter_control = "Path parameter";

constexpr std::size_t previewed_samples = 385;

// A hand-built run the panel can read nothing from but the scrub and the instant beside it: every
// sample stands at one configuration, so the run travels nowhere, the three frames are a message,
// and what the panel draws over one of these turns on the sample times and the span alone.
constexpr std::size_t stated_samples = 5;
constexpr double stated_step         = 0.25;
constexpr double stated_span         = 1.0;
constexpr std::size_t stated_scrub   = 3;

constexpr step_delta stepped{seconds{0.01}};
constexpr std::uint32_t most_steps = 100000;

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};

    return clock_source{&reading};
}

transform offset_tool_pose(const transform &pose, const transform &offset)
{
    return transform(pose * offset);
}

expected<void, refusal> recording_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &desired,
                                                     const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.25, 0.125, 7u});
    answer.solutions.push_back(configuration(desired(0, 3), desired(1, 3)));

    return {};
}

expected<joint_vector, refusal> solving_task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    return solver.ik_solve(pose, j0, solver_parameters{});
}

preview_run run_stating(double span, double moved_at, std::size_t moved)
{
    preview_run stated{span, {}, {}, {}};
    for(std::size_t which = 0; which < stated_samples; ++which)
    {
        const double at = which == moved ? moved_at : static_cast<double>(which) * stated_step;
        transform pose  = transform::Identity();
        pose(0, 3)      = static_cast<double>(which) * stated_step;

        stated.samples.push_back(preview_sample{at, trajectory::trajectory_sample{configuration(0.0, 0.0), configuration(0.0, 0.0), configuration(0.0, 0.0)}, pose});
    }

    return stated;
}

scene_robot previewing_arm()
{
    return scene_robot::compose(kinematics::compose(sliding_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{},
                                                    inverse_kinematics_ops{&recording_inverse_kinematics}, rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                        .value(),
                                robot_ops{.tool_pose_from_flange_pose = &offset_tool_pose}, rigid_motion::baseline().frame, 2u)
            .value();
}

// One arm behind one publication, one stencil drawing it, and one preview panel over both. Every
// command is posted onto the arm's own strand and drained, which is the route the composed
// application takes.
struct stage
{
    stage()
            : asked(0u)
            , frames()
            , loop(inline_workers, dictating())
            , work(loop.main_strand())
            , scene(threepp::Scene::create())
            , robot(std::make_shared<scene_robot>(previewing_arm()))
            , published(std::make_shared<arm_publisher>())
            , control(std::make_shared<robot_controller>(*robot, motion_ops{.task_space_pose = &solving_task_space_pose}, trajectory::baseline().path, task_trajectory_ops{},
                                                         trajectory::baseline().time_scaling, trajectory::baseline().trajectory, rigid_motion::screw_ops{}))
            , owned(std::make_shared<owned_arm>(work, work, robot, control, published))
            , shown(two_joint_handle(), attached_models{}, *scene, work, published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
            , panel(panel_title, published->reader(), owned, shown, [this] { ask(); })
    {
        REQUIRE(shown.initialize().has_value());
    }

    void ask()
    {
        ++asked;
        const std::weak_ptr<owned_arm> observer = owned;
        command(observer, [](robot_controller &driven, scene_robot &) { driven.preview_trajectory(configuration(0.4, -0.2)); });
    }

    // One motion through every row, whose timing is its own polynomial and which therefore stands
    // beside no scaling the arm could have chosen.
    void ask_via_points()
    {
        ++asked;
        const std::weak_ptr<owned_arm> observer = owned;
        command(observer,
                [](robot_controller &driven, scene_robot &)
                {
                    const std::vector<joint_vector> rows{configuration(0.2, 0.3), configuration(-0.1, 0.5), configuration(0.4, -0.2)};
                    driven.preview_trajectory(std::span<const joint_vector>(rows));
                });
    }

    void settle()
    {
        REQUIRE(loop.drain().has_value());
    }

    void draw()
    {
        frames.draw_frame([this] { panel.render(); });
        REQUIRE(work.post([this] { shown.render(); }).has_value());
        settle();
        scene->updateMatrixWorld(true);
    }

    void draw(std::uint32_t times)
    {
        for(std::uint32_t once = 0; once < times; ++once)
            draw();
    }

    template<typename operation>
    void commanded(operation work_to_do)
    {
        const std::weak_ptr<owned_arm> observer = owned;
        command(observer, std::move(work_to_do));
        settle();
    }

    void play_out()
    {
        for(std::uint32_t taken = 0; taken < most_steps && control->executing(); ++taken)
        {
            commanded([](robot_controller &driven, scene_robot &) { static_cast<void>(driven.advance_playback(stepped)); });
            dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        }
        REQUIRE_FALSE(control->executing());
    }

    std::shared_ptr<const preview_run> previewed()
    {
        const std::shared_ptr<const arm_snapshot> seen = published->reader().read();

        return seen == nullptr ? nullptr : seen->preview;
    }

    std::vector<Eigen::Vector3d> path(const char *named)
    {
        return line_in_world(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    threepp::Object3D *path_node(const char *named)
    {
        return first_line_under(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    std::uint32_t asked;
    imgui_frame frames;
    praxis::scheduler::scheduler loop;
    strand work;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<scene_robot> robot;
    std::shared_ptr<arm_publisher> published;
    std::shared_ptr<robot_controller> control;
    std::shared_ptr<owned_arm> owned;
    loadable_robot_stencil shown;
    trajectory_preview_window panel;
};

// One panel over one hand-built run, standing on its own publication so two of them are comparable
// frame for frame.
struct stated_stage
{
    explicit stated_stage(const preview_run &stated)
            : loop(inline_workers, dictating())
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
            , panel(panel_title, published->reader(), std::weak_ptr<owned_arm>{}, shown, std::function<void()>{})
    {
        REQUIRE(shown.initialize().has_value());

        arm_snapshot seen = at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity());
        seen.preview      = std::make_shared<const preview_run>(stated);
        published->publish(std::make_shared<const arm_snapshot>(seen));
    }

    drawing over_panel()
    {
        return over(panel);
    }

    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
    trajectory_preview_window panel;
};

ImGuiID control_named(const char *label)
{
    ImGuiWindow *const held = ImGui::FindWindowByName(panel_title);

    REQUIRE(held != nullptr);

    return held->GetID(label);
}

ImGuiID standing_on()
{
    return ImGui::GetCurrentContext()->NavId;
}

// Every item the panel offers a keyboard, walked from its first downwards. The walk ends where a
// step down leaves the cursor where it was, so the panel is what says how long it is.
std::vector<ImGuiID> controls_offered(imgui_frame &frames, const drawing &draw)
{
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_Home);

    std::vector<ImGuiID> walked;
    for(ImGuiID stood = 0; stood != standing_on();)
    {
        stood = standing_on();
        walked.push_back(stood);
        tap(frames, draw, ImGuiKey_DownArrow);
    }

    return walked;
}

// Stands the keyboard cursor on the scrub and types an index into it. A slider takes typed input
// only where it was opened with the enter key, which is what `type_at_cursor` spends.
void scrub_to(imgui_frame &frames, const drawing &draw, const char *index_text)
{
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_Home);
    for(std::size_t step = 0; step < stated_samples && standing_on() != control_named(scrub_control); ++step)
        tap(frames, draw, ImGuiKey_DownArrow);

    REQUIRE(standing_on() == control_named(scrub_control));
    type_at_cursor(frames, draw, index_text);
}

// Every vertex the panel put on screen over a run, with the scrub left where it opens or moved onto
// the sample named. Two panels differing in nothing else are comparable by this alone.
std::size_t drawn_over(const preview_run &stated, bool scrubbed)
{
    stated_stage one(stated);
    imgui_frame frames;
    const drawing draw = one.over_panel();

    frames.draw(draw);
    if(scrubbed)
        scrub_to(frames, draw, std::to_string(stated_scrub).c_str());

    frames.draw(draw);

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

const scene::plot_series &named_curve(const scene::plot_frame &frame, const std::string &name)
{
    const auto found = std::find_if(frame.series.begin(), frame.series.end(), [&name](const scene::plot_series &curve) { return curve.name == name; });

    REQUIRE(found != frame.series.end());

    return *found;
}

}

TEST_CASE("a panel rendered frame after frame asks for no preview until the route is invoked", "[manipulator][preview]")
{
    stage headless;

    headless.draw(6u);

    CHECK(headless.asked == 0u);
    CHECK(headless.previewed() == nullptr);
    CHECK(headless.panel.reading().frames.empty());

    headless.ask();
    headless.settle();

    REQUIRE(headless.previewed() != nullptr);
    CHECK(headless.previewed()->samples.size() == previewed_samples);
}

TEST_CASE("the scrub stands the arm at the sample it indexes and leaves the published solves where they were", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);

    const std::vector<std::vector<iteration_state>> before(headless.control->solves().begin(), headless.control->solves().end());
    const std::size_t indexed = 200u;

    headless.commanded([run, indexed](robot_controller &control, scene_robot &) { control.preview_joint_configuration(run->samples[indexed].motion.position); });

    CHECK(is_approx_equal(headless.robot->joint_positions(), run->samples[indexed].motion.position, 1.0e-9));
    REQUIRE(headless.control->solves().size() == before.size());
    for(std::size_t which = 0; which < before.size(); ++which)
        CHECK(headless.control->solves()[which].size() == before[which].size());
}

TEST_CASE("a scrub while a motion plays leaves the arm to the playback", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);

    headless.commanded([](robot_controller &control, scene_robot &) { control.play_preview(); });
    headless.commanded([](robot_controller &control, scene_robot &) { static_cast<void>(control.advance_playback(stepped)); });
    REQUIRE(headless.control->executing());

    const joint_vector playing = headless.robot->joint_positions();

    headless.commanded([run](robot_controller &control, scene_robot &) { control.preview_joint_configuration(run->samples.back().motion.position); });

    CHECK(is_approx_equal(headless.robot->joint_positions(), playing, 1.0e-12));
}

TEST_CASE("the two drawings stand under the strings a scene object is found by", "[manipulator][preview]")
{
    CHECK(std::string(commanded_motion_path) == "commanded");
    CHECK(std::string(traversed_motion_path) == "traversed");
}

TEST_CASE("the commanded polyline is drawn from the preview's tool poses and is cleared with the preview", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);

    headless.draw();

    CHECK(headless.path(commanded_motion_path).size() == run->samples.size());

    headless.commanded([](robot_controller &driven, scene_robot &) { driven.clear_preview(); });
    headless.settle();
    headless.draw();

    CHECK(headless.previewed() == nullptr);
    CHECK(headless.path_node(commanded_motion_path) == nullptr);
    CHECK_FALSE(headless.panel.reading().message.empty());
}

TEST_CASE("the traversed polyline stands after a playback and is left standing when the preview clears", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();
    REQUIRE(headless.previewed() != nullptr);

    headless.commanded([](robot_controller &control, scene_robot &) { control.play_preview(); });
    headless.play_out();
    headless.draw();

    const std::vector<Eigen::Vector3d> traversed = headless.path(traversed_motion_path);
    REQUIRE(traversed.size() > 1u);
    CHECK(headless.path(commanded_motion_path).size() > 1u);

    const auto *tone = headless.path_node(traversed_motion_path)->materialAs<threepp::MaterialWithColor>();
    REQUIRE(tone != nullptr);
    CHECK(tone->color.equals(threepp::Color(traversed_motion_tone)));

    headless.commanded([](robot_controller &driven, scene_robot &) { driven.clear_preview(); });
    headless.settle();
    headless.draw();

    CHECK(headless.path_node(commanded_motion_path) == nullptr);
    CHECK(headless.path(traversed_motion_path) == traversed);
}

TEST_CASE("the stencil is told a run only where the handle it was told last has changed", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();
    headless.draw();

    const threepp::Object3D *drawn = headless.path_node(commanded_motion_path);
    REQUIRE(drawn != nullptr);

    headless.draw(5u);

    CHECK(headless.path_node(commanded_motion_path) == drawn);

    headless.ask();
    headless.settle();
    headless.draw();

    CHECK(headless.path_node(commanded_motion_path) != drawn);
}

TEST_CASE("the reading carries three frames over one time axis, three curves in each and none logarithmic", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);

    const scene::plot_reading drawn = headless.panel.reading();

    CHECK(drawn.message.empty());
    CHECK(drawn.abscissa_label == "Time (s)");
    REQUIRE(drawn.frames.size() == 3u);
    CHECK(drawn.frames[0].ordinate_label == "s");
    CHECK(drawn.frames[1].ordinate_label == "ds/dt (1/s)");
    CHECK(drawn.frames[2].ordinate_label == "d2s/dt2 (1/s2)");
    for(const scene::plot_frame &frame : drawn.frames)
    {
        CHECK_FALSE(frame.logarithmic_ordinate);
        REQUIRE(frame.series.size() == 3u);
        CHECK(named_curve(frame, "Cubic").ordinate.size() == run->samples.size());
        CHECK(named_curve(frame, "Quintic (chosen)").ordinate.size() == run->samples.size());
        CHECK(named_curve(frame, "Trapezoidal").ordinate.size() == run->samples.size());
    }
}

TEST_CASE("every curve of every frame runs to the span the three scalings are drawn over", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);
    REQUIRE(run->span > 0.0);

    double longest = 0.0;
    for(const scene::plot_frame &frame : headless.panel.reading().frames)
        for(const scene::plot_series &curve : frame.series)
        {
            REQUIRE_FALSE(curve.abscissa.empty());
            CHECK(curve.abscissa.front() == 0.0);
            CHECK(is_approx_equal(curve.abscissa.back(), run->span, 1.0e-12));
            longest = std::max(longest, curve.abscissa.back());
        }

    CHECK(is_approx_equal(longest, run->span, 1.0e-12));
}

// A via-point spline could not have been commanded under a scaling it was not composed from, so the
// panel draws the one curve the motion has rather than two profiles it could never have taken.
TEST_CASE("a reading over a run standing beside no scaling carries one curve in each frame", "[manipulator][preview]")
{
    stage headless;
    headless.ask_via_points();
    headless.settle();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    REQUIRE(run != nullptr);
    REQUIRE(run->scaling.empty());
    REQUIRE_FALSE(run->parameter.empty());

    const scene::plot_reading drawn = headless.panel.reading();

    CHECK(drawn.message.empty());
    CHECK(drawn.abscissa_label == "Time (s)");
    REQUIRE(drawn.frames.size() == 3u);
    for(const scene::plot_frame &frame : drawn.frames)
    {
        REQUIRE(frame.series.size() == 1u);
        CHECK(frame.series.front().ordinate.size() == run->samples.size());
        CHECK(frame.series.front().abscissa.size() == run->samples.size());
        CHECK(frame.series.front().abscissa.front() == run->samples.front().at);
        CHECK(frame.series.front().abscissa.back() == run->samples.back().at);
    }
}

TEST_CASE("the one curve names no scaling and is marked for no choice", "[manipulator][preview]")
{
    stage headless;
    headless.ask_via_points();
    headless.settle();

    REQUIRE(headless.previewed() != nullptr);

    const scene::plot_reading drawn = headless.panel.reading();

    REQUIRE_FALSE(drawn.frames.empty());
    for(const scene::plot_frame &frame : drawn.frames)
    {
        REQUIRE(frame.series.size() == 1u);

        const std::string &name = frame.series.front().name;

        INFO("the name the one curve carries: " << name);
        for(const char *label : time_scaling_labels)
            CHECK(name.find(label) == std::string::npos);
        CHECK(name.find("chosen") == std::string::npos);
        CHECK_FALSE(name.empty());
    }
}

TEST_CASE("a reading over a run that travels nowhere carries a message and no frame", "[manipulator][preview]")
{
    const preview_run stated = run_stating(stated_span, 0.0, stated_samples);
    stated_stage one(stated);

    REQUIRE(stated.scaling.empty());
    REQUIRE(stated.parameter.empty());

    const scene::plot_reading drawn = one.panel.reading();

    CHECK(drawn.frames.empty());
    CHECK_FALSE(drawn.message.empty());
    CHECK(drawn.message != "Ask for a preview to see the motion it would play.");
}

TEST_CASE("a panel over no standing preview says what to do in place of the frames", "[manipulator][preview]")
{
    stage headless;

    const scene::plot_reading drawn = headless.panel.reading();

    CHECK(drawn.frames.empty());
    CHECK(drawn.message == "Ask for a preview to see the motion it would play.");
}

TEST_CASE("a panel told to draw no frame at all says which choice is missing", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const trajectory_preview_window silent(panel_title, headless.published->reader(), headless.owned, headless.shown, [] {}, trajectory_preview_window::settings{false, false, false});

    CHECK(silent.reading().frames.empty());
    CHECK(silent.reading().message == "Choose a curve to draw.");
}

TEST_CASE("the panel carrying a key path offers its three choices to the document and one carrying none offers nothing", "[manipulator][preview]")
{
    stage headless;

    const trajectory_preview_window unnamed(panel_title, headless.published->reader(), headless.owned, headless.shown, [] {});
    const trajectory_preview_window named_at(
            panel_title, headless.published->reader(), headless.owned, headless.shown, [] {}, trajectory_preview_window::settings{false, true, false}, "machine/trajectory_preview");

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(named_at.as_configurable() == &named_at);
    CHECK(named_at.settings_path() == "machine/trajectory_preview");
    CHECK_FALSE(named_at.state().parameter);
    CHECK(named_at.state().rate);
    CHECK_FALSE(named_at.state().rate_change);
}

// The panel plots the path parameter and offers no control over it: a queued run's parameter is not
// monotone across its legs, so the scrub indexes samples instead and is named for them.
TEST_CASE("no control the panel offers is named for the path parameter", "[manipulator][preview]")
{
    stage headless;
    headless.ask();
    headless.settle();

    const drawing draw                 = over(headless.panel);
    const std::vector<ImGuiID> offered = controls_offered(headless.frames, draw);

    REQUIRE_FALSE(offered.empty());
    CHECK(std::ranges::find(offered, control_named(scrub_control)) != offered.end());
    CHECK(std::ranges::find(offered, control_named(path_parameter_control)) == offered.end());
}

// A run that travels nowhere draws a message in place of the three frames, so the only thing left
// in the panel that reads the run's own numbers is the line beside the scrub.
TEST_CASE("the panel states the span of the run it is showing", "[manipulator][preview]")
{
    const preview_run stated = run_stating(stated_span, 0.0, stated_samples);

    CHECK(drawn_over(stated, false) == drawn_over(stated, false));
    CHECK(drawn_over(run_stating(2.0 * stated_span, 0.0, stated_samples), false) != drawn_over(stated, false));
}

// Two runs whose sample times part at one sample alone: the panel draws them alike while the scrub
// stands elsewhere and apart once it stands on that sample.
TEST_CASE("the instant the panel reports is the indexed sample's own", "[manipulator][preview]")
{
    const preview_run stated = run_stating(stated_span, 0.0, stated_samples);
    const preview_run moved  = run_stating(stated_span, 0.875, stated_scrub);

    CHECK(drawn_over(moved, false) == drawn_over(stated, false));
    CHECK(drawn_over(moved, true) != drawn_over(stated, true));
}
