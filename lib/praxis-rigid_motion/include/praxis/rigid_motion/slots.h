#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_SLOTS_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_SLOTS_H

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/slot_set.h"
#include "praxis/extension/descriptor.h"

#include <cstdint>

namespace praxis::rigid_motion {

// The enumerators are the aggregate's member names unqualified and in its declaration order: the
// capability is carried by the enumeration's own name, so a slot another extension spells the same
// way stays distinct. The trailing count enumerator names no slot; the slot set, the size assertions
// and the coverage functions all read it.
enum class frame_slot : std::uint32_t
{
    euler_from_rotation_matrix,
    rotate_x,
    rotate_y,
    rotate_z,
    rotation_matrix_from_frame_axes,
    rotation_matrix_from_euler,
    rotation_matrix_from_axis_angle,
    rotation_matrix_from_transform,
    transformation_matrix_from_position,
    transformation_matrix_from_rotation,
    transformation_matrix_from_rotation_position,
    inverse,
    count,
};

enum class screw_slot : std::uint32_t
{
    skew_symmetric,
    from_skew_symmetric,
    adjoint_matrix_from_rotation_position,
    adjoint_matrix_from_transform,
    adjoint_map,
    twist_from_angular_linear,
    twist_from_screw,
    twist_matrix_from_angular_linear,
    twist_matrix_from_twist,
    screw_axis_from_angular_linear,
    screw_axis_from_point_direction_pitch,
    matrix_exponential_so3,
    matrix_exponential_se3,
    matrix_exponential_screw,
    matrix_logarithm_so3,
    matrix_logarithm_se3_rp,
    matrix_logarithm_se3,
    count,
};

using frame_slot_set = basic_slot_set<frame_slot>;
using screw_slot_set = basic_slot_set<screw_slot>;

// A view points into the value it was given, which must outlive it. A temporary argument would leave
// the view dangling at the end of the full expression, so that call is deleted rather than diagnosed
// at run time.
capability_view view_of(const frame_ops &ops);
capability_view view_of(frame_ops &&) = delete;
capability_view view_of(const screw_ops &ops);
capability_view view_of(screw_ops &&) = delete;

}

#endif
