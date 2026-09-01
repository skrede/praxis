#include "opw_geometry.h"

#include <cartan/lie/se3.h>
#include <cartan/detail/epsilon.h>

#include <Eigen/LU>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace praxis::manipulator {

namespace {

using vector3 = Eigen::Vector3d;

constexpr std::size_t opw_joints = 6u;
constexpr double whole_turn      = 2.0 * std::numbers::pi;

// The tolerances `opw_6r_solver::make` admits a chain at, so a chain this derivation takes is one
// the solver takes.
constexpr double direction_tolerance   = cartan::detail::sqrt_epsilon_v<double>;
constexpr double position_tolerance    = cartan::default_verification_tolerance_v<double>.position();
constexpr double orientation_tolerance = cartan::default_verification_tolerance_v<double>.orientation();

// Six angles distinct in magnitude and in sign at every joint, so a length, a frame or a sign the
// derivation read wrongly moves the reconstruction instead of cancelling inside it.
constexpr std::array<std::array<double, opw_joints>, 5> probed_configurations{{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
        {-0.3, 0.5, -0.7, 0.9, -1.1, 1.3},
        {0.7, -0.4, 1.1, -0.2, 0.8, -0.6},
        {1.2, 0.9, -0.5, 1.4, -0.9, 0.3},
}};

// A joint axis as the line it fixes: its direction, and the point on it nearest the frame's origin.
struct axis_line
{
    vector3 direction;
    vector3 through;
};

// The frame the decomposition is written in. `upward` is the base axis, `sideways` the shoulder
// axis, and `forward` the direction the arm reaches at the configuration the chain describes, which
// is what makes the first joint's angle zero there.
struct basis
{
    vector3 forward;
    vector3 sideways;
    vector3 upward;
};

signed char matching(const vector3 &axis, const vector3 &canonical)
{
    return axis.dot(canonical) < 0.0 ? static_cast<signed char>(-1) : static_cast<signed char>(1);
}

vector3 across_from(const vector3 &along, const vector3 &normal)
{
    return along - along.dot(normal) * normal;
}

std::optional<std::array<axis_line, opw_joints>> to_axis_lines(const screw_chain &chain)
{
    if(chain.space_screws.size() != opw_joints)
        return std::nullopt;

    std::array<axis_line, opw_joints> lines{};
    for(std::size_t at = 0; at < opw_joints; ++at)
    {
        const vector3 direction = chain.space_screws[at].head<3>();
        if(std::abs(direction.norm() - 1.0) > direction_tolerance)
            return std::nullopt;
        lines[at] = axis_line{direction, direction.cross(chain.space_screws[at].tail<3>())};
    }

    return lines;
}

// The point the last three axes meet at in the least-squares sense, absent when they meet at none:
// the residual measured against each line is the distance from that line to the point.
std::optional<vector3> wrist_centre(const std::array<axis_line, opw_joints> &lines)
{
    Eigen::Matrix3d spread = Eigen::Matrix3d::Zero();
    vector3 weighted       = vector3::Zero();
    for(std::size_t at = 3; at < opw_joints; ++at)
    {
        const Eigen::Matrix3d perpendicular = Eigen::Matrix3d::Identity() - lines[at].direction * lines[at].direction.transpose();
        spread += perpendicular;
        weighted += perpendicular * lines[at].through;
    }

    const vector3 centre = spread.fullPivLu().solve(weighted);
    for(std::size_t at = 3; at < opw_joints; ++at)
        if(across_from(centre - lines[at].through, lines[at].direction).norm() > position_tolerance)
            return std::nullopt;

    return centre;
}

// The base axis points the way the wrist stands from the base and the shoulder axis the way that
// leaves the arm reaching forward; those two choices fix the frame, and with it the sign correction
// of every joint below.
std::optional<basis> to_basis(const std::array<axis_line, opw_joints> &lines, const vector3 &wrist)
{
    if(std::abs(lines[0].direction.dot(lines[1].direction)) >= direction_tolerance)
        return std::nullopt;
    if(std::abs(lines[1].direction.dot(lines[2].direction)) <= 1.0 - direction_tolerance)
        return std::nullopt;

    const vector3 reach  = wrist - lines[0].through;
    const vector3 upward = reach.dot(lines[0].direction) < 0.0 ? vector3(-lines[0].direction) : lines[0].direction;
    const vector3 turned = across_from(reach, upward).dot(lines[1].direction.cross(upward)) < 0.0 ? vector3(-lines[1].direction) : lines[1].direction;

    return basis{turned.cross(upward), turned, upward};
}

// The shoulder's two offsets from the base axis and the two link lengths above it, together with the
// angle the upper arm stands at in the configuration the chain describes.
struct upper_arm
{
    double a1;
    double b;
    double c1;
    double c2;
    double angle;
};

upper_arm to_upper_arm(const std::array<axis_line, opw_joints> &lines, const basis &frame, const vector3 &wrist)
{
    const vector3 shoulder = lines[1].through - lines[0].through;
    const vector3 link     = across_from(lines[2].through - lines[1].through, frame.sideways);

    return upper_arm{shoulder.dot(frame.forward), (wrist - lines[0].through).dot(frame.sideways), shoulder.dot(frame.upward), link.norm(),
                     std::atan2(link.dot(frame.forward), link.dot(frame.upward))};
}

// The elbow-to-wrist span carries the forearm length and the elbow offset together, and the roll
// axis is what separates them: it stands along the forearm, so the angle between the two is the
// offset's. A forearm of negative length is the same span read through the opposite roll direction.
struct forearm
{
    double a2;
    double c3;
    double angle;
    double bearing;
    signed char sign;
};

forearm to_forearm(const std::array<axis_line, opw_joints> &lines, const basis &frame, const vector3 &wrist, double shoulder_angle)
{
    const vector3 span = across_from(wrist - lines[2].through, frame.sideways);
    const double reach = std::atan2(span.dot(frame.forward), span.dot(frame.upward));
    const vector3 roll = lines[3].direction;

    double bearing   = std::atan2(roll.dot(frame.forward), roll.dot(frame.upward));
    signed char sign = 1;
    if(std::cos(std::remainder(reach - bearing, whole_turn)) < 0.0)
    {
        sign    = -1;
        bearing = std::atan2(-roll.dot(frame.forward), -roll.dot(frame.upward));
    }
    const double lean = std::remainder(reach - bearing, whole_turn);

    return forearm{span.norm() * std::sin(lean), span.norm() * std::cos(lean), std::remainder(bearing - shoulder_angle, whole_turn), bearing, sign};
}

basis wrist_basis(const basis &frame, double bearing)
{
    const vector3 approach = std::sin(bearing) * frame.forward + std::cos(bearing) * frame.upward;

    return basis{std::cos(bearing) * frame.forward - std::sin(bearing) * frame.upward, frame.sideways, approach};
}

// The three wrist angles are read off the axes themselves rather than decomposed out of the flange
// orientation, so an arm whose home pose holds the wrist straight -- where a decomposition fixes
// only the sum of the first and the last -- still answers with the two the axes carry.
struct wrist_turn
{
    double c4;
    std::array<double, 3> angles;
    signed char roll;
    signed char flange;
};

wrist_turn to_wrist_turn(const std::array<axis_line, opw_joints> &lines, const basis &held, const transform &home, const vector3 &centre)
{
    const rotation orientation = home.block<3, 3>(0, 0);
    const vector3 approach     = orientation.col(2);
    const signed char roll     = matching(lines[4].direction, held.sideways);
    const vector3 turned       = static_cast<double>(roll) * lines[4].direction;
    const double fourth        = std::atan2(-turned.dot(held.forward), turned.dot(held.sideways));
    const double fifth         = std::atan2(approach.dot(held.forward) * std::cos(fourth) + approach.dot(held.sideways) * std::sin(fourth), approach.dot(held.upward));

    rotation frame;
    frame << held.forward, held.sideways, held.upward;
    const rotation unturned = Eigen::AngleAxisd(-fifth, vector3::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(-fourth, vector3::UnitZ()).toRotationMatrix();
    const rotation residual = unturned * frame.transpose() * orientation;

    return wrist_turn{(home.block<3, 1>(0, 3) - centre).dot(approach), {fourth, fifth, std::atan2(residual(1, 0), residual(0, 0))}, roll, matching(lines[5].direction, approach)};
}

// The one place praxis's own terms are laid out in the field order and the sign convention the
// dependency's aggregate is read under.
cartan::opw_parameters<double> assembled(const std::array<axis_line, opw_joints> &lines, const basis &frame, const upper_arm &arm, const forearm &reach, const wrist_turn &wrist)
{
    return cartan::opw_parameters<double>{arm.a1,
                                          reach.a2,
                                          arm.b,
                                          arm.c1,
                                          arm.c2,
                                          reach.c3,
                                          wrist.c4,
                                          {0.0, -arm.angle, -reach.angle, -wrist.angles[0], -wrist.angles[1], -wrist.angles[2]},
                                          {matching(lines[0].direction, frame.upward), matching(lines[1].direction, frame.sideways), matching(lines[2].direction, frame.sideways),
                                           reach.sign, wrist.roll, wrist.flange}};
}

}

expected<cartan::opw_parameters<double>, refusal> to_opw_parameters(const screw_chain &chain)
{
    const std::optional<std::array<axis_line, opw_joints>> lines = to_axis_lines(chain);
    if(!lines.has_value())
        return unexpected(refusal::unsupported_input);

    const std::optional<vector3> centre = wrist_centre(*lines);
    if(!centre.has_value())
        return unexpected(refusal::unsupported_input);

    const std::optional<basis> frame = to_basis(*lines, *centre);
    if(!frame.has_value())
        return unexpected(refusal::unsupported_input);

    const upper_arm arm    = to_upper_arm(*lines, *frame, *centre);
    const forearm reach    = to_forearm(*lines, *frame, *centre, arm.angle);
    const wrist_turn wrist = to_wrist_turn(*lines, wrist_basis(*frame, reach.bearing), chain.home, *centre);

    return assembled(*lines, *frame, arm, reach, wrist);
}

expected<void, refusal> agrees_with_chain(const forward_kinematics_ops &forward, const screw_chain &chain, const cartan::opw_parameters<double> &parameters)
{
    for(const std::array<double, opw_joints> &probed : probed_configurations)
    {
        const Eigen::Vector<double, 6> at          = Eigen::Map<const Eigen::Vector<double, 6>>(probed.data());
        const expected<transform, refusal> reached = forward.forward_kinematics(chain.home, chain.space_screws, joint_vector(at));
        if(!reached)
            return unexpected(reached.error());

        const auto against = cartan::se3<double>::from_matrix(*reached);
        if(!against.has_value())
            return unexpected(refusal::degenerate);

        const twist residual = (cartan::opw_forward(parameters, at).inverse() * against.value()).log();
        if(residual.head<3>().norm() > orientation_tolerance || residual.tail<3>().norm() > position_tolerance)
            return unexpected(refusal::unsupported_input);
    }

    return {};
}

refusal refusal_from(cartan::analytical_failure failure)
{
    switch(failure)
    {
        case cartan::analytical_failure::degenerate_geometry:
            return refusal::unsupported_input;
        case cartan::analytical_failure::unreachable:
        case cartan::analytical_failure::singular_configuration:
        case cartan::analytical_failure::verification_failed:
            return refusal::no_solution;
        case cartan::analytical_failure::non_finite_input:
            return refusal::degenerate;
    }

    return refusal::degenerate;
}

}
