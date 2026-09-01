#include "fixtures.h"
#include "drawn_chain.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <cstddef>
#include <numbers>

using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

namespace {

// What a configuration read back off a played-out motion is comparable at, and how far the dictated
// reading is raised per service: one service covers every period the raise spans.
constexpr double reached_at = 1.0e-6;
constexpr seconds serviced{0.05};

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

// Wide enough that a configuration a full turn from where the arm stands is a motion the bounds
// shape rather than one they forbid, and quick enough that it plays out in a handful of services.
screw_chain reaching_chain()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 20.0);
    bounds.acceleration   = joint_vector::Constant(2, 80.0);
    bounds.lower_position = joint_vector::Constant(2, -10.0);
    bounds.upper_position = joint_vector::Constant(2, 10.0);

    return screw_chain(praxis::transform::Identity(), {praxis::screw_axis::Zero(), praxis::screw_axis::Zero()}, bounds);
}

// Three answers for one target, in an order that is not their order of nearness to where the arm
// stands: the second is the nearest and the first is what taking the front of the set would give.
praxis::expected<void, praxis::refusal> three_branches(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const praxis::transform &,
                                                       const joint_vector &, const solver_parameters &, ik_result &answer)
{
    answer.solutions.push_back(configuration(1.0, 1.0));
    answer.solutions.push_back(configuration(0.5, -0.25));
    answer.solutions.push_back(configuration(-1.0, 1.25));

    return {};
}

// One answer names the posture the arm already holds a full turn along the first joint and the other
// a fraction of a turn away. Wrapped, the first is the nearer; unwrapped it is the farther by a wide
// margin, so which the arm takes says which metric decided.
praxis::expected<void, praxis::refusal> a_turn_and_a_fraction(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const praxis::transform &,
                                                              const joint_vector &, const solver_parameters &, ik_result &answer)
{
    answer.solutions.push_back(configuration(0.25 + 2.0 * std::numbers::pi, -0.5));
    answer.solutions.push_back(configuration(0.9, -0.5));

    return {};
}

std::vector<praxis::screw_axis> two_axes()
{
    const praxis::rigid_motion::screw_ops screw = praxis::rigid_motion::baseline().screw;

    return {screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0).value(),
            screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0), Eigen::Vector3d::UnitZ(), 0.0).value()};
}

// The arm on a strand and a stencil reading the same publication, which is the whole path a solve
// travels: a command on the strand, a publication, and a composition telling the drawing what the
// publication carried.
struct stage
{
    explicit stage(const inverse_kinematics_ops &inverse)
            : loop(inline_workers, dictating())
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , seen(published->reader())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                    praxis::rigid_motion::screw_slot_set{})
    {
        auto solver = kinematics::compose(reaching_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{}, inverse,
                                          praxis::rigid_motion::baseline().screw, praxis::rigid_motion::baseline().frame);
        auto driven = std::make_shared<scene_robot>(scene_robot::compose(solver.value(), robot_ops{}, praxis::rigid_motion::baseline().frame, 2u).value());
        driven->set_joint_positions(configuration(0.25, -0.5));

        auto control = std::make_shared<robot_controller>(*driven, motion_ops{}, composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                                          praxis::rigid_motion::screw_ops{});

        arm = std::make_shared<owned_arm>(loop.main_strand(), loop.main_strand(), driven, control, published);
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    }

    // One start is the multi-start command at a set of one, which is what these cases are about: the
    // path from a solve to the drawing, not the several starts the seeded set carries. The velocity
    // factor is raised first so the motion the solve commands plays out in a handful of services; the
    // service bound is a failure report rather than a synchronization device.
    void solve_toward(double x)
    {
        praxis::transform target = praxis::transform::Identity();
        target(0, 3)             = x;

        const std::vector<joint_vector> one_start{configuration(0.0, 0.0)};

        command(std::weak_ptr<owned_arm>(arm), [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
        REQUIRE(loop.drain().has_value());
        command(std::weak_ptr<owned_arm>(arm), [&target, &one_start](robot_controller &control, scene_robot &) { control.solve_from_seeds(target, one_start); });
        REQUIRE(loop.drain().has_value());

        for(int service = 0; service < 400 && seen.read()->executing; ++service)
        {
            dictated += std::chrono::duration_cast<time_point::duration>(serviced);
            REQUIRE(loop.drain().has_value());
        }
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    std::size_t items_of(std::size_t figure, const named_by_index &named)
    {
        std::size_t found = 0;
        while(chain_part(*scene, loadable_robot_stencil::solution_figure_name(figure), named(found)) != nullptr)
            ++found;

        return found;
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    arm_reader seen;
    loadable_robot_stencil shown;
    std::shared_ptr<owned_arm> arm;
};

// The composition's share of the drawing: every answered configuration except the one the arm was
// driven to, which the rendered arm is already standing at.
std::vector<joint_vector> beside(const arm_snapshot &seen)
{
    std::vector<joint_vector> apart;
    for(const joint_vector &answered : seen.solutions)
        if(!praxis::is_approx_equal(answered, seen.joints, reached_at))
            apart.push_back(answered);

    return apart;
}

}

TEST_CASE("a solve on the arm's strand publishes every answer, takes the nearest of them and leaves the rest to be drawn", "[manipulator][solutions]")
{
    stage headless(inverse_kinematics_ops{&three_branches});

    headless.solve_toward(0.5);

    const std::shared_ptr<const arm_snapshot> seen = headless.seen.read();

    REQUIRE_FALSE(seen->executing);
    REQUIRE(seen->solutions.size() == 3u);
    CHECK(praxis::is_approx_equal(seen->solutions[0], configuration(1.0, 1.0), reached_at));
    CHECK(praxis::is_approx_equal(seen->solutions[2], configuration(-1.0, 1.25), reached_at));

    // The one start of this command reached the answer the solver named first, which is the front of
    // the set the fold left.
    REQUIRE(seen->reached.size() == 1u);
    CHECK(seen->reached[0] == 0u);

    // The nearest under the wrapped metric, and not the front of the set, which is what an
    // enumeration order would have given.
    CHECK(praxis::is_approx_equal(seen->joints, configuration(0.5, -0.25), reached_at));

    const std::vector<joint_vector> others = beside(*seen);
    REQUIRE(others.size() == 2u);
    REQUIRE(headless.shown.set_solution_figures(others).has_value());
    headless.draw();

    CHECK(chain_node(*headless.scene, loadable_robot_stencil::solution_figure_name(0)) != nullptr);
    CHECK(chain_node(*headless.scene, loadable_robot_stencil::solution_figure_name(1)) != nullptr);
    CHECK(chain_node(*headless.scene, loadable_robot_stencil::solution_figure_name(2)) == nullptr);

    for(std::size_t figure = 0; figure < others.size(); ++figure)
    {
        CHECK(headless.items_of(figure, [figure](std::size_t at) { return loadable_robot_stencil::solution_segment_name(figure, at); }) == 3u);
        CHECK(headless.items_of(figure, [figure](std::size_t joint) { return loadable_robot_stencil::solution_mark_name(figure, joint); }) == 2u);
    }
}

TEST_CASE("an answer a full turn from where the arm stands is the nearest of the answers, so the arm keeps the posture it had", "[manipulator][solutions]")
{
    stage headless(inverse_kinematics_ops{&a_turn_and_a_fraction});

    headless.solve_toward(0.5);

    const std::shared_ptr<const arm_snapshot> seen = headless.seen.read();

    REQUIRE_FALSE(seen->executing);
    REQUIRE(seen->solutions.size() == 2u);
    CHECK(praxis::is_approx_equal(seen->joints, configuration(0.25 + 2.0 * std::numbers::pi, -0.5), reached_at));

    // The posture is the one the arm opened at, which is what wrapping each joint difference to a
    // half turn either way is for; the fraction of a turn away is the farther of the two.
    CHECK(std::abs(std::fmod(seen->joints[0] - 0.25, 2.0 * std::numbers::pi)) < reached_at);
}
