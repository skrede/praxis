#include "fixtures.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/compat/detail/callable.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

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

constexpr seconds chunk{0.25};
constexpr std::uint32_t most_chunks = 64;

// What a case reaches the aggregate through. There is no share here: the observer is weak and the
// only share left is the one the release route carries, which is what the retirement is handed.
struct arm_pipe
{
    strand work;
    arm_reader seen;
    std::weak_ptr<owned_arm> observer;
    praxis::detail::move_only_function<void()> release;
};

arm_pipe pipe(praxis::scheduler::scheduler &loop)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto control   = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                              praxis::trajectory::trajectory_ops{}, praxis::rigid_motion::screw_ops{});

    auto owned = std::make_shared<owned_arm>(work, work, driven, control, published);

    return arm_pipe{work, published->reader(), owned, [owned = std::move(owned)]() mutable { owned.reset(); }};
}

praxis::transform planar_pose(double x, double y)
{
    praxis::transform pose = praxis::transform::Identity();
    pose(0, 3)             = x;
    pose(1, 3)             = y;

    return pose;
}

void advance(praxis::scheduler::scheduler &loop, seconds by)
{
    dictated += std::chrono::duration_cast<time_point::duration>(by);
    REQUIRE(loop.drain().has_value());
}

// The velocity factor is raised first so the motion spans a countable number of chunks rather than
// the three and a third times as many the default factor would take.
void start_motion(praxis::scheduler::scheduler &loop, const arm_pipe &arm)
{
    command(arm.observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
    command(arm.observer, [](robot_controller &control, scene_robot &) { control.task_space_ptp(planar_pose(0.9, -0.9)); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->executing);
}

using window_share = std::shared_ptr<praxis::scene::imgui_window>;

// What a placement is watched through. Every observer here is weak, because a share held by the case
// would keep alive the very thing the case is asking about.
struct placement
{
    std::weak_ptr<threepp::Object3D> robot;
    std::weak_ptr<threepp::Object3D> tool;
    std::weak_ptr<threepp::Object3D> world;
    std::weak_ptr<arm_publisher> carried;
};

praxis::scene::window_route ignoring()
{
    return [](const window_share &) {};
}

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

// The three models an arm places, composed here rather than read out of a description so that each
// one has a share the case can observe weakly. The publisher is what the release callable carries,
// which is state of the arm's own strand rather than of the scene.
std::shared_ptr<praxis::scene::preset> placing_arm(const praxis::scene::preset_site &site, placement &watched)
{
    const std::shared_ptr<threepp::Robot> handle   = two_joint_handle();
    const std::shared_ptr<threepp::Object3D> tool  = threepp::Object3D::create();
    const std::shared_ptr<threepp::Object3D> world = threepp::Object3D::create();
    auto published                                 = std::make_shared<arm_publisher>();

    auto body  = std::make_shared<loadable_robot_stencil>(handle, attached_models{tool, world}, site.scene, site.render, published->reader(), praxis::rigid_motion::baseline().screw,
                                                          praxis::rigid_motion::screw_slot_set{});
    auto built = std::make_shared<praxis::scene::preset>(body, std::vector<window_share>{}, site.add_window, site.remove_window);

    watched           = placement{handle, tool, world, published};
    built->release_cb = [held = std::move(published)]() mutable { held.reset(); };

    return built;
}

// A placement standing in a scene with the preset held here rather than by a composition, so that
// what the withdrawal does is readable apart from what dropping the preset does.
struct standing
{
    standing()
            : watched()
            , loop(inline_workers, dictating())
            , scene(threepp::Scene::create())
            , composed(placing_arm(praxis::scene::preset_site{*scene, loop.main_strand(), *loop.make_strand(), nullptr, ignoring(), ignoring(), {}}, watched))
    {
        REQUIRE(composed->initialize().has_value());
    }

    const loadable_robot_stencil &body() const
    {
        return dynamic_cast<const loadable_robot_stencil &>(*composed->stencil);
    }

    placement watched;
    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<praxis::scene::preset> composed;
};

}

// The defect this pins: a composition unloaded while a motion plays used to free the arm under a
// playback still touching it. The freeing point is the retirement acknowledgment, so the whole
// assertion is when a weak observer expires, on a dictated clock with no sanitizer and no timing.
TEST_CASE("a composition unloaded while a motion plays frees nothing until the acknowledgment runs", "[manipulator][ownership]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    arm_pipe arm = pipe(loop);

    const std::weak_ptr<owned_arm> observer = arm.observer;

    start_motion(loop, arm);
    advance(loop, chunk);
    REQUIRE(arm.seen.read()->executing);

    REQUIRE(loop.retire_strand(arm.work, std::move(arm.release)).has_value());
    REQUIRE_FALSE(observer.expired());

    REQUIRE(loop.drain().has_value());

    REQUIRE(observer.expired());
}

TEST_CASE("the acknowledgment frees behind everything the strand already held", "[manipulator][ownership]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    arm_pipe arm = pipe(loop);

    const std::weak_ptr<owned_arm> observer = arm.observer;
    std::vector<int> order;

    command(observer, [&order](robot_controller &, scene_robot &) { order.push_back(0); });
    command(observer, [&order](robot_controller &, scene_robot &) { order.push_back(1); });
    REQUIRE(order.empty());

    REQUIRE(loop.retire_strand(arm.work,
                               [&order, release = std::move(arm.release)]() mutable
                               {
                                   release();
                                   order.push_back(9);
                               })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(order == std::vector<int>{0, 1, 9});
    REQUIRE(observer.expired());
}

TEST_CASE("a post issued after the retirement is refused and reaches nothing", "[manipulator][ownership]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop);

    const std::shared_ptr<const arm_snapshot> last = arm.seen.read();

    REQUIRE(loop.retire_strand(arm.work, nullptr).has_value());

    const praxis::expected<void, rejection> late = arm.work.post([] {});

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::strand_retired);

    command(arm.observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.9); });
    REQUIRE(loop.drain().has_value());

    REQUIRE_FALSE(arm.observer.expired());
    REQUIRE(arm.seen.read() == last);
    REQUIRE(last->velocity_factor == 0.3);
}

// A task no longer scheduled reports no tallies, so the receipt is empty exactly when a student
// would want the lateness of the motion just played. The final tick copies them into the
// publication, and a publication made afterwards still carries them -- which is what says they were
// copied rather than read back from a receipt that is gone.
TEST_CASE("the counters of the motion just played are readable after it has ended", "[manipulator][ownership]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop);

    start_motion(loop, arm);
    for(std::uint32_t taken = 0; taken < most_chunks && arm.seen.read()->executing; ++taken)
        advance(loop, chunk);

    const std::shared_ptr<const arm_snapshot> ended = arm.seen.read();

    REQUIRE_FALSE(ended->executing);
    REQUIRE(ended->motion.ran > 0);
    REQUIRE(ended->motion.dropped == 0);
    REQUIRE(ended->motion.policy == overrun::catch_up);

    command(arm.observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> after = arm.seen.read();

    REQUIRE(after != ended);
    REQUIRE(after->velocity_factor == 0.5);
    REQUIRE(after->motion.ran == ended->motion.ran);
}

// The three cases below read one node each rather than a descendant count, because a count cannot
// say which of the three withdrawals fired. Each locks its observer to read the node and lets the
// lock go again, so the share that outlives the withdrawal is the stencil's and not the case's.
TEST_CASE("the robot node leaves the scene at the withdrawal and the stencil still holds it", "[manipulator][ownership]")
{
    standing stage;

    REQUIRE(stage.watched.robot.lock()->parent == stage.scene.get());

    stage.composed->tear_down();

    REQUIRE_FALSE(stage.watched.robot.expired());
    REQUIRE(stage.watched.robot.lock()->parent == nullptr);
    REQUIRE(stage.watched.robot.lock().get() == static_cast<const threepp::Object3D *>(&stage.body().robot()));
}

TEST_CASE("the tool node leaves the scene at the withdrawal and the stencil still holds it", "[manipulator][ownership]")
{
    standing stage;

    REQUIRE(stage.watched.tool.lock()->parent == stage.scene.get());

    stage.composed->tear_down();

    REQUIRE_FALSE(stage.watched.tool.expired());
    REQUIRE(stage.watched.tool.lock()->parent == nullptr);
    REQUIRE(stage.body().attached_at(flange_attachment::tool) == stage.watched.tool.lock());
}

TEST_CASE("the world reference object leaves the scene at the withdrawal and the stencil still holds it", "[manipulator][ownership]")
{
    standing stage;

    REQUIRE(stage.watched.world.lock()->parent == stage.scene.get());

    stage.composed->tear_down();

    REQUIRE_FALSE(stage.watched.world.expired());
    REQUIRE(stage.watched.world.lock()->parent == nullptr);
    REQUIRE(stage.body().world_object() == stage.watched.world.lock());
}

// Where each half of a placement's ownership ends. The scene graph and the shares of it belong to the
// strand the frames are drawn on, and both end together when the unload drops the preset; what the
// arm's own strand was driving is handed to the retirement instead and outlives the unload that
// queued it.
TEST_CASE("an unload ends the placement and the acknowledgment ends what the arm's strand was driving", "[manipulator][ownership]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    praxis::scene::composition held(*target, loop, {});
    placement watched;

    held.windows_through(ignoring(), ignoring());

    const std::size_t bare = descendants(*target);

    REQUIRE(held.load([&watched](const praxis::scene::preset_site &site) { return placing_arm(site, watched); }).has_value());

    REQUIRE(descendants(*target) > bare);
    REQUIRE(watched.robot.lock()->parent == target.get());
    REQUIRE(watched.tool.lock()->parent == target.get());
    REQUIRE(watched.world.lock()->parent == target.get());

    held.unload();

    REQUIRE(descendants(*target) == bare);
    REQUIRE(watched.robot.expired());
    REQUIRE(watched.tool.expired());
    REQUIRE(watched.world.expired());
    REQUIRE_FALSE(watched.carried.expired());

    REQUIRE(loop.drain().has_value());

    REQUIRE(watched.carried.expired());
}
