#include "fixtures.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/control_configuration.h"
#include "praxis/manipulator/control_parameters_window.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <fstream>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::string_view parameters_at = "machine/parameters";

struct arm_pipe
{
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

arm_pipe pipe(scheduler::scheduler &loop, const trajectory::time_scaling_ops &scaling)
{
    const scheduler::strand work = *loop.make_strand();
    const auto driven            = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto control           = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, scaling, trajectory::trajectory_ops{},
                                                                      rigid_motion::screw_ops{}, rigid_motion::frame_ops{});
    const auto published         = std::make_shared<arm_publisher>();

    return arm_pipe{std::make_shared<owned_arm>(work, work, driven, control, published), published->reader()};
}

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_control_parameters(shape, parameters_at);

    return shape;
}

control_parameters_window::settings authored(std::string_view body)
{
    const scratch_tree tree("control_parameters_window");
    const std::filesystem::path file = tree.root() / "parameters.xml";
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(file, tree.root()), config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return read_control_parameters(answered.values, parameters_at);
}

void held_trapezoidal(robot_controller &control, scene_robot &)
{
    control.set_velocity_factor(0.5);
    control.set_time_scaling(time_scaling_choice::trapezoidal);
    control.set_trapezoid_bounds(path_parameter_bounds{2.5, 10.0});
}

bool bounded_at(const std::optional<path_parameter_bounds> &held_to, double rate, double rate_change)
{
    return held_to.has_value() && held_to->max_rate == rate && held_to->max_rate_change == rate_change;
}

}

TEST_CASE("a window opened from a document hands the arm the rate, scaling and bounds it shows", "[manipulator][configuration]")
{
    const control_parameters_window::settings read = authored("<parameters velocity_factor=\"0.625\" time_scaling=\"Cubic\" path_rate=\"2.5\" path_rate_change=\"10\"/>");
    REQUIRE(read.scaling == time_scaling_choice::cubic);

    scheduler::scheduler loop(scheduler::inline_workers);
    const arm_pipe arm = pipe(loop, composing_time_scaling());
    const control_parameters_window window("Control parameters", arm.seen, arm.owned, read);
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> published = arm.seen.read();
    REQUIRE(published != nullptr);
    CHECK(published->time_scaling == time_scaling_choice::cubic);
    CHECK(published->velocity_factor == 0.625);
    CHECK(bounded_at(published->trapezoid, 2.5, 10.0));
    CHECK(window.state().scaling == published->time_scaling);
}

TEST_CASE("an arm opened at the cubic by its window previews under the cubic", "[manipulator][configuration]")
{
    const control_parameters_window::settings read = authored("<parameters time_scaling=\"Cubic\"/>");
    REQUIRE(read.scaling == time_scaling_choice::cubic);

    scheduler::scheduler loop(scheduler::inline_workers);
    const arm_pipe arm = pipe(loop, trajectory::time_scaling_ops{.cubic = trajectory::baseline().time_scaling.cubic});
    const control_parameters_window window("Control parameters", arm.seen, arm.owned, read);
    REQUIRE(loop.drain().has_value());

    command(arm.owned, [](robot_controller &control, scene_robot &) { control.preview_trajectory(configuration(0.5, 0.25)); });
    REQUIRE(loop.drain().has_value());

    CHECK(arm.seen.read()->preview != nullptr);
}

TEST_CASE("a window opened with no settings shows what the arm holds and leaves it there", "[manipulator][configuration]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    const arm_pipe arm = pipe(loop, composing_time_scaling());
    command(arm.owned, &held_trapezoidal);
    REQUIRE(loop.drain().has_value());

    const control_parameters_window window("Control parameters", arm.seen, arm.owned);
    REQUIRE(loop.drain().has_value());

    const control_parameters_window::settings shown = window.state();
    CHECK(shown.velocity == 0.5f);
    CHECK(shown.scaling == time_scaling_choice::trapezoidal);
    CHECK(bounded_at(shown.trapezoid, 2.5, 10.0));

    const std::shared_ptr<const arm_snapshot> published = arm.seen.read();
    CHECK(published->velocity_factor == 0.5);
    CHECK(published->time_scaling == time_scaling_choice::trapezoidal);
    CHECK(bounded_at(published->trapezoid, 2.5, 10.0));
}
