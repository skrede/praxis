#include "screw_modeling_padding.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/Geometry>

#include <span>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

screw_modeling_window::parameterization typed_as(const screw_axis &screw)
{
    return screw.head<3>().norm() <= angular_epsilon ? screw_modeling_window::parameterization::angular_linear : screw_modeling_window::parameterization::point_direction_pitch;
}

screw_axis supplied_or_opening(const rigid_motion::screw_ops &turning, const screw_axis &derived, const supplied_screw &held)
{
    return held ? *held : screw_modeling_window::opening_screw(turning, derived);
}

std::vector<screw_axis> as_drawn(const screw_chain &derived, const rigid_motion::screw_ops &turning, std::span<const supplied_screw> supplied)
{
    std::vector<screw_axis> drawn;
    drawn.reserve(derived.joint_count());
    for(std::size_t joint = 0u; joint < derived.joint_count(); ++joint)
        drawn.push_back(supplied_or_opening(turning, derived.space_screws[joint], joint < supplied.size() ? supplied[joint] : supplied_screw()));

    return drawn;
}

// A turning joint's `(z, 0)` and a translating joint's `(0, z)` are both what the published
// construction answers for that axis, so neither is written out here as a literal.
screw_axis screw_modeling_window::opening_screw(const rigid_motion::screw_ops &turning, const screw_axis &derived)
{
    if(typed_as(derived) == parameterization::angular_linear)
        return turning.screw_axis_from_angular_linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

    const expected<screw_axis, refusal> built = turning.screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0);

    return built ? built.value() : screw_axis::Zero();
}

}
