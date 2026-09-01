#include "fixtures.h"
#include "captured_log.h"
#include "three_link_arm.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/SVD>
#include <Eigen/Core>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <limits>
#include <cstdint>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

namespace {

time_point reading()
{
    return time_point{};
}

// Six rows and one column per joint, the rows scaled apart so the top three and the bottom three
// carry different singular values and a decomposition taken over the wrong rows is visible.
praxis::expected<jacobian, praxis::refusal> answering_space_jacobian(std::span<const praxis::screw_axis>, const joint_vector &theta)
{
    jacobian columns(6, theta.size());
    for(Eigen::Index joint = 0; joint < theta.size(); ++joint)
        for(Eigen::Index row = 0; row < 6; ++row)
            columns(row, joint) = std::cos(0.25 * static_cast<double>(row) + static_cast<double>(joint) + theta[joint]) * (row < 3 ? 1.0 : 4.0);

    return columns;
}

// The value the mathematics cannot be taken over reaches the decomposition through a slot that
// answered, so nothing above it has refused and this is the only place the fault can be named.
praxis::expected<jacobian, praxis::refusal> deranged_space_jacobian(std::span<const praxis::screw_axis> screws, const joint_vector &theta)
{
    jacobian columns = answering_space_jacobian(screws, theta).value();
    columns(1, 0)    = std::numeric_limits<double>::quiet_NaN();

    return columns;
}

differential_kinematics_ops answering_jacobian()
{
    return differential_kinematics_ops{.space_jacobian = &answering_space_jacobian};
}

differential_kinematics_ops deranged_jacobian()
{
    return differential_kinematics_ops{.space_jacobian = &deranged_space_jacobian};
}

struct arm_pipe
{
    strand work;
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

// A chain of the named width standing on the module's own inert robot bindings: nothing here needs a
// graphics context, and the width is what decides whether an ellipsoid can be taken at all.
arm_pipe pipe(scheduler &loop, const screw_chain &chain, std::uint32_t joints, const differential_kinematics_ops &differential)
{
    const strand work = *loop.make_strand();
    auto robot        = std::make_shared<scene_robot>(
            scene_robot::compose(kinematics::compose(chain, forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential, inverse_kinematics_ops{},
                                                     praxis::rigid_motion::baseline().screw, praxis::rigid_motion::baseline().frame)
                                         .value(),
                                 robot_ops{}, praxis::rigid_motion::baseline().frame, joints)
                    .value());

    auto controller = std::make_shared<robot_controller>(*robot, motion_ops{}, composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                                         praxis::rigid_motion::screw_ops{});
    auto published  = std::make_shared<arm_publisher>();
    auto owned      = std::make_shared<owned_arm>(work, work, robot, controller, published);

    return arm_pipe{work, owned, published->reader()};
}

Eigen::Vector3d values_of(const Eigen::MatrixXd &block)
{
    return Eigen::JacobiSVD<Eigen::MatrixXd>(block).singularValues().head<3>();
}

constexpr double exact = 1.0e-12;

}

// The published ellipsoid is compared against a decomposition of the published matrix rather than
// against a remembered number, so an entry landing at the wrong index of the aggregate is caught.
TEST_CASE("a published snapshot carries the ellipsoids of the Jacobians published beside them", "[manipulator][manipulability]")
{
    scheduler loop(inline_workers, clock_source{&reading});
    const arm_pipe arm                             = pipe(loop, three_link_arm(), 3u, answering_jacobian());
    const std::shared_ptr<const arm_snapshot> seen = arm.seen.read();

    REQUIRE(seen != nullptr);
    REQUIRE(seen->space_jacobian.has_value());
    REQUIRE(seen->space_manipulability.angular.has_value());
    REQUIRE(seen->space_manipulability.linear.has_value());

    const Eigen::Vector3d angular = values_of(seen->space_jacobian->topRows(3));
    const Eigen::Vector3d linear  = values_of(seen->space_jacobian->bottomRows(3));

    CHECK((seen->space_manipulability.angular->singular_values - angular).cwiseAbs().maxCoeff() < exact);
    CHECK((seen->space_manipulability.linear->singular_values - linear).cwiseAbs().maxCoeff() < exact);
    CHECK(std::abs(seen->space_manipulability.angular->principal_axes.determinant() - 1.0) < exact);
}

// The body slot is unbound here, so the refusal reaching both of its ellipsoids is the Jacobian's own
// and is answered for where the Jacobian is taken rather than named a second time.
TEST_CASE("a Jacobian slot that is unbound publishes its own refusal for both ellipsoids and reports nothing", "[manipulator][manipulability]")
{
    scheduler loop(inline_workers, clock_source{&reading});
    std::shared_ptr<const arm_snapshot> seen;

    const std::string reported = reported_by(
            [&loop, &seen]
            {
                const arm_pipe arm = pipe(loop, three_link_arm(), 3u, answering_jacobian());
                seen               = arm.seen.read();
            });

    REQUIRE(seen != nullptr);
    CHECK_FALSE(seen->body_manipulability.angular.has_value());
    CHECK_FALSE(seen->body_manipulability.linear.has_value());
    CHECK(seen->body_manipulability.angular.error() == praxis::refusal::not_implemented);
    CHECK(reported.empty());
}

// Every block of a chain this narrow is rank-deficient at every configuration, and thirty-odd suites
// stand an arm of exactly this width, so what it publishes is a refusal and not a nonsense number.
// A width the arm simply has is said in the publication and not in the log.
TEST_CASE("a chain narrower than three joints publishes a refusal for all four ellipsoids and reports nothing", "[manipulator][manipulability]")
{
    scheduler loop(inline_workers, clock_source{&reading});
    std::shared_ptr<const arm_snapshot> seen;

    const std::string reported = reported_by(
            [&loop, &seen]
            {
                const arm_pipe arm = pipe(loop, sliding_chain(), 2u, answering_jacobian());
                seen               = arm.seen.read();
            });

    REQUIRE(seen != nullptr);
    CHECK_FALSE(seen->space_manipulability.angular.has_value());
    CHECK_FALSE(seen->space_manipulability.linear.has_value());
    CHECK(seen->space_manipulability.angular.error() == praxis::refusal::unsupported_input);
    CHECK(seen->space_manipulability.linear.error() == praxis::refusal::unsupported_input);
    CHECK(seen->body_manipulability.linear.error() == praxis::refusal::not_implemented);
    CHECK(reported.empty());
}

// The latch is the decomposition's own: a standing fault is named once however many publications
// stand behind it, and the pose refusal beside it keeps its own.
TEST_CASE("a Jacobian no ellipsoid can be built from is named once and not once per publication", "[manipulator][manipulability]")
{
    scheduler loop(inline_workers, clock_source{&reading});
    std::shared_ptr<const arm_snapshot> seen;

    const std::string reported = reported_by(
            [&loop, &seen]
            {
                const arm_pipe arm = pipe(loop, three_link_arm(), 3u, deranged_jacobian());
                command(std::weak_ptr<owned_arm>(arm.owned), [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
                REQUIRE(loop.drain().has_value());
                seen = arm.seen.read();
            });

    REQUIRE(seen != nullptr);
    CHECK(seen->space_jacobian.has_value());
    CHECK_FALSE(seen->space_manipulability.angular.has_value());
    CHECK(seen->space_manipulability.angular.error() == praxis::refusal::degenerate);
    CHECK(reported.find("dk.manipulability") != std::string::npos);
    CHECK(reported.find("dk.manipulability") == reported.rfind("dk.manipulability"));
}
