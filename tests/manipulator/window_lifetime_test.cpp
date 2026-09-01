#include "fixtures.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/control_parameters_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <stdexcept>

using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

namespace {

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

std::shared_ptr<robot_controller> controlling(scene_robot &driven)
{
    return std::make_shared<robot_controller>(driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                              praxis::rigid_motion::screw_ops{});
}

struct arm_pipe
{
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

// Nothing but the aggregate holds the robot, the controller or the publisher, so dropping its last
// share destroys all three and what the reader goes on reading is the cell they left behind.
arm_pipe pipe(scheduler &loop)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto control   = controlling(*driven);
    const auto published = std::make_shared<arm_publisher>();

    return arm_pipe{std::make_shared<owned_arm>(work, work, driven, control, published), published->reader()};
}

}

// What is left to refuse is a holder that keeps a share: the share is what keeps the referent alive
// for the holder's whole life, so there is nothing to hold and nothing to defer the decision to. A
// window keeps none, so there is no window here.
TEST_CASE("a holder given no handle to hold refuses to be built")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = std::make_shared<arm_publisher>();

    CHECK_THROWS_AS(loadable_robot_stencil(nullptr, attached_models{}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                           praxis::rigid_motion::screw_slot_set{}),
                    std::invalid_argument);
}

// The frame between an unload and a window leaving the list: the aggregate has been freed by the
// retirement acknowledgment while the window is still there to be drawn.
TEST_CASE("a window whose observer has expired posts nothing and reads the publication it last saw")
{
    scheduler loop(inline_workers, dictating());
    arm_pipe arm = pipe(loop);

    const std::weak_ptr<owned_arm> observer = arm.owned;
    const control_parameters_window window("Control parameters", arm.seen, observer);

    const std::shared_ptr<const arm_snapshot> last = arm.seen.read();
    arm.owned.reset();
    REQUIRE(observer.expired());

    REQUIRE_NOTHROW(command(observer, [](robot_controller &commanded, scene_robot &) { commanded.set_velocity_factor(0.5); }));
    REQUIRE(loop.drain().has_value());

    CHECK(arm.seen.read() == last);
    CHECK(window.state().velocity == static_cast<float>(last->velocity_factor));
}
