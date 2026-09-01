#include "window_stage.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/joint_curve_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr const char *panel_title = "Joint curves";
constexpr const char *curves_at   = "machine/joint_curves";

// The rate a run is at rest to, which is the tolerance the reference suite reads a via-point run's
// ends at. The curves are drawn in degrees, so the rate the plot carries is that many times larger.
constexpr double at_rest_radians = 1.0e-9;

// The rate below which a run through the rows would be pausing at one of them rather than passing
// through it, in the degrees a second the plot is drawn in. A run that does pause reaches exactly
// zero there, so what this separates is a run that never slows below it from one that stops.
constexpr double passing_through = 0.5;

// Four rows inside the fixture's position bounds, no two of them alike and no two of them a
// degenerate segment apart.
std::vector<joint_vector> authored_rows()
{
    return {configuration(-1.0, 0.5), configuration(0.0, -0.8), configuration(1.0, 0.6), configuration(0.2, -0.4)};
}

// One arm behind one publication and one plot over it. Every command is posted onto the arm's own
// strand and drained, which is the route the composed application takes.
struct stage
{
    stage()
            : frames()
            , loop(inline_workers)
            , work(loop.main_strand())
            , robot(std::make_shared<scene_robot>(two_joint_arm(robot_ops{})))
            , published(std::make_shared<arm_publisher>())
            , control(std::make_shared<robot_controller>(*robot, motion_ops{}, trajectory::baseline().path, task_trajectory_ops{}, trajectory::baseline().time_scaling,
                                                         trajectory::baseline().trajectory, rigid_motion::screw_ops{}))
            , owned(std::make_shared<owned_arm>(work, work, robot, control, published))
            , panel(panel_title, published->reader(), joint_curve_window::settings{}, curves_at)
    {
    }

    template<typename operation>
    void commanded(operation asked)
    {
        const std::weak_ptr<owned_arm> observer = owned;
        command(observer, std::move(asked));
        REQUIRE(loop.drain().has_value());
    }

    // One motion through every row, which is what the polynomial the rows are fitted with runs.
    void ask_through_the_rows()
    {
        const std::vector<joint_vector> rows = authored_rows();
        commanded([rows](robot_controller &driven, scene_robot &) { driven.preview_trajectory(std::span<const joint_vector>(rows)); });
    }

    // The same rows run as a motion apiece, coming fully to rest at each of them.
    void ask_between_the_rows()
    {
        const std::vector<joint_vector> rows = authored_rows();
        commanded([rows](robot_controller &driven, scene_robot &) { driven.preview_in_turn(std::span<const joint_vector>(rows)); });
    }

    std::shared_ptr<const preview_run> previewed() const
    {
        const std::shared_ptr<const arm_snapshot> seen = published->reader().read();

        return seen == nullptr ? nullptr : seen->preview;
    }

    imgui_frame frames;
    praxis::scheduler::scheduler loop;
    strand work;
    std::shared_ptr<scene_robot> robot;
    std::shared_ptr<arm_publisher> published;
    std::shared_ptr<robot_controller> control;
    std::shared_ptr<owned_arm> owned;
    joint_curve_window panel;
};

bool carries_curve(const scene::plot_frame &frame, const std::string &name)
{
    return std::any_of(frame.series.begin(), frame.series.end(), [&name](const scene::plot_series &curve) { return curve.name == name; });
}

// How fast the whole configuration is moving at each drawn instant, taken off the rate frame so that
// what is read is what a learner sees rather than what the arm published beneath it.
std::vector<double> drawn_speeds(const scene::plot_reading &shown)
{
    REQUIRE(shown.frames.size() == 3u);

    const scene::plot_frame &rate = shown.frames[1];
    REQUIRE_FALSE(rate.series.empty());

    std::vector<double> speeds(rate.series.front().ordinate.size(), 0.0);
    for(const scene::plot_series &curve : rate.series)
        for(std::size_t at = 0; at < speeds.size() && at < curve.ordinate.size(); ++at)
            speeds[at] = std::hypot(speeds[at], curve.ordinate[at]);

    return speeds;
}

double slowest_inside(const std::vector<double> &speeds)
{
    REQUIRE(speeds.size() > 2u);

    return *std::min_element(speeds.begin() + 1, speeds.end() - 1);
}

}

TEST_CASE("the reading carries three frames over one time axis, one curve per joint, and none logarithmic", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const scene::plot_reading shown = headless.panel.reading();

    CHECK(shown.message.empty());
    CHECK(shown.abscissa_label == "Time (s)");
    REQUIRE(shown.frames.size() == 3u);
    CHECK(shown.frames[0].ordinate_label == "theta (deg)");
    CHECK(shown.frames[1].ordinate_label == "dtheta/dt (deg/s)");
    CHECK(shown.frames[2].ordinate_label == "d2theta/dt2 (deg/s2)");

    for(const scene::plot_frame &frame : shown.frames)
    {
        INFO(frame.ordinate_label);
        CHECK_FALSE(frame.logarithmic_ordinate);
        REQUIRE(frame.series.size() == 2u);
        CHECK(carries_curve(frame, "j1"));
        CHECK(carries_curve(frame, "j2"));
    }
}

TEST_CASE("the abscissa of every curve runs from zero to the previewed span", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const std::shared_ptr<const preview_run> run = headless.previewed();
    const scene::plot_reading shown              = headless.panel.reading();

    REQUIRE(run != nullptr);
    REQUIRE(run->span > 0.0);
    REQUIRE(shown.frames.size() == 3u);

    for(const scene::plot_frame &frame : shown.frames)
        for(const scene::plot_series &curve : frame.series)
        {
            INFO(curve.name);
            REQUIRE(curve.abscissa.size() == run->samples.size());
            CHECK(curve.abscissa.front() == 0.0);
            CHECK(curve.abscissa.back() == run->span);
        }
}

// The switch is found by the name the curve it governs is drawn under rather than by where it stands
// in the panel, so a panel that reorders its controls does not quietly leave this passing.
TEST_CASE("a joint switched off in the panel is absent from all three frames and the others stay", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const drawing draw = [&headless] { headless.panel.render(); };
    start_navigating(headless.frames, draw);
    reach(headless.frames, draw, ImGuiKey_Home);
    tap(headless.frames, draw, ImGuiKey_Space);

    const scene::plot_reading shown = headless.panel.reading();

    REQUIRE(headless.panel.state().hidden == std::vector<std::size_t>{0u});
    REQUIRE(shown.frames.size() == 3u);
    for(const scene::plot_frame &frame : shown.frames)
    {
        INFO(frame.ordinate_label);
        CHECK(frame.series.size() == 1u);
        CHECK_FALSE(carries_curve(frame, "j1"));
        CHECK(carries_curve(frame, "j2"));
    }
}

TEST_CASE("a joint a document left out is absent from all three frames", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const joint_curve_window without_the_second(panel_title, headless.published->reader(), joint_curve_window::settings{{1u}}, curves_at);
    const scene::plot_reading shown = without_the_second.reading();

    REQUIRE(shown.frames.size() == 3u);
    for(const scene::plot_frame &frame : shown.frames)
    {
        INFO(frame.ordinate_label);
        CHECK(frame.series.size() == 1u);
        CHECK(carries_curve(frame, "j1"));
        CHECK_FALSE(carries_curve(frame, "j2"));
    }
}

// This is where the velocity continuity a run through via points is built for is a fact rather than
// a claim: the rate the plot draws comes back to zero at both ends and at no instant between them,
// where the same rows run as separate motions do come to rest inside the run.
TEST_CASE("the drawn rate returns to zero at the ends of a run through the rows and at no instant inside it", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const std::vector<double> through = drawn_speeds(headless.panel.reading());

    CHECK(through.front() < at_rest_radians * degrees_per_radian);
    CHECK(through.back() < at_rest_radians * degrees_per_radian);
    CHECK(slowest_inside(through) > passing_through);

    headless.ask_between_the_rows();

    const std::vector<double> between = drawn_speeds(headless.panel.reading());

    CHECK(slowest_inside(between) < passing_through);
}

// A run through the rows is one motion and a run between them is one per pair, which is the whole
// difference between the two ways the same rows are played.
TEST_CASE("a run through the rows is drawn as one motion where a run between them is drawn as one per pair", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const std::shared_ptr<const preview_run> through = headless.previewed();

    REQUIRE(through != nullptr);

    const double through_span      = through->span;
    const std::size_t through_size = through->samples.size();

    headless.ask_between_the_rows();

    const std::shared_ptr<const preview_run> between = headless.previewed();

    REQUIRE(between != nullptr);
    CHECK(between->samples.size() == (authored_rows().size() - 1u) * through_size);
    CHECK(through_span < between->span);
}

TEST_CASE("a reading with no preview standing says what to do in place of the frames", "[manipulator][joint-curves]")
{
    const stage headless;
    const scene::plot_reading shown = headless.panel.reading();

    CHECK(shown.message == "Ask for a preview to see how each joint moves through it.");
    CHECK(shown.frames.empty());
}

TEST_CASE("a plot told to draw no joint at all says which choice is missing", "[manipulator][joint-curves]")
{
    stage headless;
    headless.ask_through_the_rows();

    const joint_curve_window silent(panel_title, headless.published->reader(), joint_curve_window::settings{{0u, 1u}}, curves_at);

    CHECK(silent.reading().message == "Choose a joint to draw.");
    CHECK(silent.reading().frames.empty());
}

TEST_CASE("the window carrying a key path offers the joints it hides and one carrying none offers nothing", "[manipulator][joint-curves]")
{
    const stage headless;
    const joint_curve_window unnamed(panel_title, headless.published->reader());
    const joint_curve_window named_at(panel_title, headless.published->reader(), joint_curve_window::settings{{1u, 0u}}, curves_at);

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(unnamed.settings_path().empty());
    CHECK(unnamed.state().hidden.empty());
    CHECK(named_at.as_configurable() == &named_at);
    CHECK(named_at.settings_path() == curves_at);
    CHECK(named_at.state().hidden == std::vector<std::size_t>{0u, 1u});
}
