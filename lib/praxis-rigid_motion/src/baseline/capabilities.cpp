#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <array>

namespace praxis::rigid_motion {

namespace {

constexpr frame_ops bound_frame{
        .euler_from_rotation_matrix                   = &euler_from_rotation_matrix,
        .rotate_x                                     = &rotate_x,
        .rotate_y                                     = &rotate_y,
        .rotate_z                                     = &rotate_z,
        .rotation_matrix_from_frame_axes              = &rotation_matrix_from_frame_axes,
        .rotation_matrix_from_euler                   = &rotation_matrix_from_euler,
        .rotation_matrix_from_axis_angle              = &rotation_matrix_from_axis_angle,
        .rotation_matrix_from_transform               = &rotation_matrix_from_transform,
        .transformation_matrix_from_position          = &transformation_matrix_from_position,
        .transformation_matrix_from_rotation          = &transformation_matrix_from_rotation,
        .transformation_matrix_from_rotation_position = &transformation_matrix_from_rotation_position,
        .inverse                                      = &inverse,
};

constexpr screw_ops bound_screw{
        .skew_symmetric                        = &skew_symmetric,
        .from_skew_symmetric                   = &from_skew_symmetric,
        .adjoint_matrix_from_rotation_position = &adjoint_matrix_from_rotation_position,
        .adjoint_matrix_from_transform         = &adjoint_matrix_from_transform,
        .adjoint_map                           = &adjoint_map,
        .twist_from_angular_linear             = &twist_from_angular_linear,
        .twist_from_screw                      = &twist_from_screw,
        .twist_matrix_from_angular_linear      = &twist_matrix_from_angular_linear,
        .twist_matrix_from_twist               = &twist_matrix_from_twist,
        .screw_axis_from_angular_linear        = &screw_axis_from_angular_linear,
        .screw_axis_from_point_direction_pitch = &screw_axis_from_point_direction_pitch,
        .matrix_exponential_so3                = &matrix_exponential_so3,
        .matrix_exponential_se3                = &matrix_exponential_se3,
        .matrix_exponential_screw              = &matrix_exponential_screw,
        .matrix_logarithm_so3                  = &matrix_logarithm_so3,
        .matrix_logarithm_se3_rp               = &matrix_logarithm_se3_rp,
        .matrix_logarithm_se3                  = &matrix_logarithm_se3,
};

}

capabilities baseline()
{
    return capabilities{.frame = bound_frame, .screw = bound_screw};
}

std::array<capability_view, 2> capability_views(const capabilities &c)
{
    return {view_of(c.frame), view_of(c.screw)};
}

}
