#include "evaluation_cases.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

// Radians per second, radians per second squared, and the ends of the position interval in radians.
// The interval covers a full turn either way of zero, which is wider than any angle `angle_radians`
// draws, so no drawn configuration ever stands outside the chain's own bounds.
constexpr double joint_speed_limit        = 10.0;
constexpr double joint_acceleration_limit = 100.0;
constexpr double joint_travel_limit       = 6.3;

joint_limits limits_over(std::size_t joints)
{
    const auto count = static_cast<Eigen::Index>(joints);

    return joint_limits{
            .velocity       = joint_vector::Constant(count, joint_speed_limit),
            .acceleration   = joint_vector::Constant(count, joint_acceleration_limit),
            .lower_position = joint_vector::Constant(count, -joint_travel_limit),
            .upper_position = joint_vector::Constant(count, joint_travel_limit),
    };
}

meios::link<> link_named(std::string name)
{
    meios::link<> record{};
    record.name = std::move(name);

    return record;
}

meios::joint<> revolute(std::size_t index, evaluation::case_source &drawn)
{
    const Eigen::Vector3d direction = drawn.unit_direction();
    const Eigen::Vector3d offset    = drawn.position_metres();
    const Eigen::Vector3d turn      = drawn.euler_triple_radians();

    meios::joint<> record{};
    record.name               = "joint" + std::to_string(index + 1u);
    record.kind               = meios::joint_kind::revolute;
    record.parent             = index == 0u ? "base" : "link" + std::to_string(index);
    record.child              = "link" + std::to_string(index + 1u);
    record.axis               = {direction.x(), direction.y(), direction.z()};
    record.origin.translation = {offset.x(), offset.y(), offset.z()};
    record.origin.rotation    = {turn.x(), turn.y(), turn.z()};

    return record;
}

}

joint_vector drawn_joints(evaluation::case_source &drawn, std::size_t joints)
{
    joint_vector theta(static_cast<Eigen::Index>(joints));

    for(Eigen::Index axis = 0; axis < theta.size(); ++axis)
        theta[axis] = drawn.angle_radians();

    return theta;
}

evaluation_case drawn_case(evaluation::case_source &drawn)
{
    const std::size_t joints = drawn.axis_count();
    const transform home     = drawn.transform_member();

    std::vector<screw_axis> screws;
    screws.reserve(joints);
    for(std::size_t axis = 0; axis < joints; ++axis)
        screws.push_back(drawn.unit_twist());

    return evaluation_case{screw_chain(home, std::move(screws), limits_over(joints)), drawn_joints(drawn, joints)};
}

std::optional<solve_case> drawn_solve(evaluation::case_source &drawn)
{
    const evaluation_case example = drawn_case(drawn);
    const joint_vector seed       = drawn_joints(drawn, example.chain.joint_count());
    expected<kinematics, refusal> solver =
            kinematics::compose(example.chain, baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
    if(!solver)
        return std::nullopt;

    const expected<transform, refusal> reached = solver->fk_solve(example.joints);
    if(!reached)
        return std::nullopt;

    return solve_case{std::move(*solver), *reached, example.joints, seed};
}

std::optional<transform> reached_by(const solve_case &solved, const joint_vector &answer)
{
    const expected<transform, refusal> pose = solved.solver.fk_solve(answer);
    if(!pose)
        return std::nullopt;

    return *pose;
}

meios::model<> drawn_model(evaluation::case_source &drawn)
{
    const std::size_t joints = drawn.axis_count();

    meios::model<> model;
    model.name  = "drawn_serial_machine";
    model.links = {link_named("base")};
    model.topo  = meios::robot_topology{{-1}, {-1}, {0}, {0}};

    for(std::size_t axis = 0u; axis < joints; ++axis)
    {
        model.joints.push_back(revolute(axis, drawn));
        model.links.push_back(link_named("link" + std::to_string(axis + 1u)));
        model.topo.parent_of.push_back(static_cast<int>(axis));
        model.topo.joint_of.push_back(static_cast<int>(axis));
        model.topo.order.push_back(static_cast<int>(axis + 1u));
    }

    return model;
}

}
