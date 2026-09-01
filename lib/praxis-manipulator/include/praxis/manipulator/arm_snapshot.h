#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ARM_SNAPSHOT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ARM_SNAPSHOT_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/scheduler/overrun.h"
#include "praxis/scheduler/snapshot.h"

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

// One instant of a previewed motion. The time is from the motion's own start, and the pose is taken
// at the configuration this same entry carries.
struct preview_sample
{
    double at;
    trajectory::trajectory_sample motion;
    expected<transform, refusal> tool_pose;
};

// A whole previewed motion, sampled once and read many times. The scaling curves stand beside the
// samples rather than inside them: one curve per scaling in the choice enumeration's own order, each
// sampled at the same times as the samples, and none at all for a motion whose timing is its own
// polynomial.
struct preview_run
{
    double span;
    std::vector<preview_sample> samples;
    std::vector<std::vector<trajectory::scaling_sample>> scaling;
    // The path parameter the motion itself realizes: the distance travelled along the
    // configuration-space path it traces, normalized to the whole and dimensionless, with its two
    // derivatives with respect to time, sampled at the samples' own times. A motion composed from a
    // chosen scaling realizes that scaling, so this repeats the chosen curve above; a motion whose
    // timing is its own polynomial answers for no choice and this is the only curve it has. Lynch &
    // Park, Modern Robotics, sec. 9.2.
    std::vector<trajectory::scaling_sample> parameter;
};

// The ellipsoid one 3-by-n block of a Jacobian traces out over the unit ball of joint rates. The
// semi-axis lengths stand in descending order and the columns of the basis are the axes they lie
// along, expressed in the space frame and right-handed. An angular block's lengths are per unit
// joint rate and a linear block's are metres per unit joint rate. The measure is the product of the
// three lengths and the condition number is the first over the last, absent where the last is zero.
// Lynch & Park, Modern Robotics, section 5.4.
struct manipulability_ellipsoid
{
    Eigen::Vector3d singular_values;
    Eigen::Matrix3d principal_axes;
    double measure;
    std::optional<double> condition;
};

// The two ellipsoids one Jacobian carries: the angular one over its top three rows and the linear
// one over its bottom three. Lynch & Park, Modern Robotics, section 5.4.
struct jacobian_manipulability
{
    expected<manipulability_ellipsoid, refusal> angular;
    expected<manipulability_ellipsoid, refusal> linear;
};

// The whole readable state of the arm as one value, immutable once published and read whole, so a
// reader can never pair a pose taken from one publication with joints taken from another. Every
// orientation is a rotation matrix and every pose is expressed in the space frame.
struct arm_snapshot
{
    joint_vector joints;
    joint_limits limits;
    transform tool_offset;
    expected<transform, refusal> tool_pose;
    expected<transform, refusal> flange_pose;
    expected<Eigen::Vector3d, refusal> tool_position;
    expected<Eigen::Vector3d, refusal> flange_position;
    expected<rotation, refusal> tool_orientation;
    expected<rotation, refusal> flange_orientation;
    recording_parameters recording;
    double velocity_factor;
    bool executing;
    scheduler::task_counters motion;
    // One sequence per request the most recent requesting command handed the solver and the solver
    // entered, in the order the requests were made. A publication whose own command handed the solver
    // no request carries what the last requesting one left.
    std::vector<std::vector<iteration_state>> iterations;
    // Both are taken at the configuration this same publication carries.
    expected<jacobian, refusal> space_jacobian;
    expected<jacobian, refusal> body_jacobian;
    // Each is taken over the Jacobian published above it and at the same configuration.
    jacobian_manipulability space_manipulability;
    jacobian_manipulability body_manipulability;
    // The distinct configurations the most recent requesting command's solves answered, in the order
    // they were first met and taken against the target that command was given. Empty where the last
    // requesting command reached no solver and where one did and the solver answered nothing.
    std::vector<joint_vector> solutions;
    // Which of those each of that command's starts reached, one entry per sequence above and in that
    // same order. An entry equal to the number of configurations names a start that entered the solve
    // and reached none of them. Empty where the sequences are.
    std::vector<std::size_t> reached;
    // The motion last sampled ahead of being played, and null where no preview stands. Replaced
    // whole and never appended to, so a reader holding a share reads one motion end to end.
    std::shared_ptr<const preview_run> preview;
    // The tool poses the arm has stood at since the run in flight began, in the order they were
    // reached, spanning every target of a queued run, and null where no run stands. Replaced whole
    // under the same rule as the preview.
    std::shared_ptr<const std::vector<transform>> traversed;
    // What the arm holds rather than what the motion in flight was composed under, so a reader sees
    // the choice the next motion will take.
    time_scaling_choice time_scaling;
    // Absent leaves a trapezoidal motion at the bounds the motion's own extent gives.
    std::optional<path_parameter_bounds> trapezoid;
};

// Which of the distinct configurations above the arm itself stands at: the nearest of them to the
// configuration the same publication carries, under the joint distance the commands take, and
// nothing where the publication carries none or where their width is not the arm's. A reader
// pairing the configurations with the arm asks this rather than comparing them itself, so what
// nearness means stays said in one place.
std::optional<std::size_t> solution_at(const arm_snapshot &seen);

using arm_publisher = scheduler::snapshot_publisher<std::shared_ptr<const arm_snapshot>>;
using arm_reader    = scheduler::snapshot_reader<std::shared_ptr<const arm_snapshot>>;

}

#endif
