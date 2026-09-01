#include "fixtures.h"

#include "imgui_frame.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/control_parameters_window.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <functional>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

const praxis::rigid_motion::frame_ops reference = praxis::rigid_motion::baseline().frame;

const char *const absent_line = "The arm has published nothing yet.";

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

// Only whether a publication is there at all is under test, so the value one carries is what an arm
// at rest reports: no joints, every position at the origin and every orientation upright.
arm_snapshot at_rest()
{
    const praxis::transform identity = praxis::transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const praxis::rotation upright(praxis::rotation::Identity());

    return arm_snapshot{joint_vector(),
                        joint_limits{},
                        identity,
                        identity,
                        identity,
                        origin,
                        origin,
                        upright,
                        upright,
                        recording_parameters{},
                        1.0,
                        false,
                        task_counters{},
                        {},
                        praxis::unexpected(praxis::refusal::not_implemented),
                        praxis::unexpected(praxis::refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

std::shared_ptr<arm_publisher> publishing()
{
    auto published = std::make_shared<arm_publisher>();
    published->publish(std::make_shared<const arm_snapshot>(at_rest()));

    return published;
}

std::shared_ptr<robot_controller> controlling(scene_robot &driven)
{
    return std::make_shared<robot_controller>(driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                              praxis::rigid_motion::screw_ops{});
}

using drawing = std::function<void()>;

int geometry_of(const drawing &draw)
{
    imgui_frame frames;
    frames.draw(draw);

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.command_lists() >= 1);

    return frames.vertices();
}

// A panel drawn over an absent publication is the panel those titles would carry with that one
// sentence in each and nothing else, so what the two frames drew is the whole comparison.
int stating_absence(const std::vector<std::string> &titles)
{
    return geometry_of(
            [&titles]
            {
                for(const std::string &title : titles)
                {
                    ImGui::Begin(title.c_str());
                    ImGui::TextUnformatted(absent_line);
                    ImGui::End();
                }
            });
}

// A publication withdrawn under a live window is the route to the render path's own answer: a window
// is refused where it is built, so the reader it keeps has published by the time it can be drawn.
// What it then owes is one panel under its own title carrying one sentence, so a second panel or
// anything drawn beside that sentence answers differently.
template<typename window, typename... built_from>
void states_absence_alone(const char *title, built_from &&...rest)
{
    const std::shared_ptr<arm_publisher> published = publishing();
    window panel(title, published->reader(), std::weak_ptr<owned_arm>(), std::forward<built_from>(rest)...);
    published->publish(nullptr);

    CHECK(geometry_of([&panel] { panel.render(); }) == stating_absence({title}));
}

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

}

TEST_CASE("a control parameters window built over an arm that has published nothing refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(control_parameters_window("Control parameters", unheld.reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the control parameters window was given no published arm state to hold"));
}

TEST_CASE("a trajectory recording window built over an arm that has published nothing refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(trajectory_recording_window("Trajectory recording", unheld.reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the trajectory recording window was given no published arm state to hold"));
}

TEST_CASE("a joint control window built over an arm that has published nothing refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(joint_control_window("Joint control", unheld.reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the joint control window was given no published arm state to hold"));
}

TEST_CASE("a control parameters panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<control_parameters_window>("Control parameters");
}

TEST_CASE("a trajectory recording panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<trajectory_recording_window>("Trajectory recording");
}

TEST_CASE("a joint control panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<joint_control_window>("Joint control");
}

TEST_CASE("a task space panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<task_space_window>("Task space", reference, std::make_shared<edited_pose>());
}

TEST_CASE("a tool frame jog panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<tool_jog_window>("Tool frame jog", reference, std::make_shared<edited_pose>());
}

TEST_CASE("a screw jog panel left with no publication draws its account of that", "[manipulator]")
{
    states_absence_alone<screw_jog_window>("Screw jog", reference, std::make_shared<edited_pose>());
}

// Nothing is published to mirror and there is no one on a strand to say it to, so what the stencil
// owes is to leave the node exactly as it stands.
TEST_CASE("a stencil whose arm has published nothing leaves the rendered robot as it stands", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    arm_publisher unheld;

    loadable_robot_stencil shown(two_joint_handle(), attached_models{}, *target, loop.main_strand(), unheld.reader(), praxis::rigid_motion::baseline().screw,
                                 praxis::rigid_motion::screw_slot_set{});
    REQUIRE(shown.initialize().has_value());

    const std::size_t standing            = descendants(*target);
    const std::vector<float> before_frame = shown.robot().jointValues();

    REQUIRE(loop.main_strand().post([&shown] { shown.render(); }).has_value());
    REQUIRE(loop.drain().has_value());

    CHECK(descendants(*target) == standing);
    CHECK(shown.robot().jointValues() == before_frame);
}

// The shape a preset composes in, with the two lines that hold its ordering taken the other way
// round: the reader is handed over before the state whose construction publishes exists.
TEST_CASE("a composition handing out a reader before its arm state publishes is refused by name", "[manipulator]")
{
    scheduler loop(inline_workers, dictating());
    const strand work                              = *loop.make_strand();
    const std::shared_ptr<scene_robot> driven      = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const std::shared_ptr<robot_controller> keeper = controlling(*driven);
    const std::shared_ptr<arm_publisher> published = std::make_shared<arm_publisher>();

    REQUIRE_THROWS_MATCHES(control_parameters_window("Control parameters", published->reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the control parameters window was given no published arm state to hold"));

    const std::shared_ptr<owned_arm> owned = std::make_shared<owned_arm>(work, work, driven, keeper, published);

    REQUIRE_NOTHROW(control_parameters_window("Control parameters", published->reader(), owned));
}
