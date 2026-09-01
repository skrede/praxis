#include "praxis/rigid_motion/slots.h"

#include <array>
#include <cstddef>

namespace praxis::rigid_motion {

namespace {

// An identical-code-folding linker can merge two byte-identical function bodies to one address. That
// can only report a slot as still holding its default when the supplied implementation is
// byte-identical to the inert one, in which case the report is correct.
constexpr std::array frame_descriptors{
        slot_descriptor{"frame.euler_from_rotation_matrix",
                        [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->euler_from_rotation_matrix == &inert::euler_from_rotation_matrix; }},
        slot_descriptor{"frame.rotate_x", [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotate_x == &inert::rotate_x; }},
        slot_descriptor{"frame.rotate_y", [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotate_y == &inert::rotate_y; }},
        slot_descriptor{"frame.rotate_z", [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotate_z == &inert::rotate_z; }},
        slot_descriptor{"frame.rotation_matrix_from_frame_axes",
                        [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotation_matrix_from_frame_axes == &inert::rotation_matrix_from_frame_axes; }},
        slot_descriptor{"frame.rotation_matrix_from_euler",
                        [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotation_matrix_from_euler == &inert::rotation_matrix_from_euler; }},
        slot_descriptor{"frame.rotation_matrix_from_axis_angle",
                        [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotation_matrix_from_axis_angle == &inert::rotation_matrix_from_axis_angle; }},
        slot_descriptor{"frame.rotation_matrix_from_transform",
                        [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->rotation_matrix_from_transform == &inert::rotation_matrix_from_transform; }},
        slot_descriptor{"frame.transformation_matrix_from_position", [](const void *value) -> bool
                        { return static_cast<const frame_ops *>(value)->transformation_matrix_from_position == &inert::transformation_matrix_from_position; }},
        slot_descriptor{"frame.transformation_matrix_from_rotation", [](const void *value) -> bool
                        { return static_cast<const frame_ops *>(value)->transformation_matrix_from_rotation == &inert::transformation_matrix_from_rotation; }},
        slot_descriptor{"frame.transformation_matrix_from_rotation_position", [](const void *value) -> bool
                        { return static_cast<const frame_ops *>(value)->transformation_matrix_from_rotation_position == &inert::transformation_matrix_from_rotation_position; }},
        slot_descriptor{"frame.inverse", [](const void *value) -> bool { return static_cast<const frame_ops *>(value)->inverse == &inert::inverse; }},
};

constexpr std::array screw_descriptors{
        slot_descriptor{"screw.skew_symmetric", [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->skew_symmetric == &inert::skew_symmetric; }},
        slot_descriptor{"screw.from_skew_symmetric",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->from_skew_symmetric == &inert::from_skew_symmetric; }},
        slot_descriptor{"screw.adjoint_matrix_from_rotation_position", [](const void *value) -> bool
                        { return static_cast<const screw_ops *>(value)->adjoint_matrix_from_rotation_position == &inert::adjoint_matrix_from_rotation_position; }},
        slot_descriptor{"screw.adjoint_matrix_from_transform",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->adjoint_matrix_from_transform == &inert::adjoint_matrix_from_transform; }},
        slot_descriptor{"screw.adjoint_map", [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->adjoint_map == &inert::adjoint_map; }},
        slot_descriptor{"screw.twist_from_angular_linear",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->twist_from_angular_linear == &inert::twist_from_angular_linear; }},
        slot_descriptor{"screw.twist_from_screw", [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->twist_from_screw == &inert::twist_from_screw; }},
        slot_descriptor{"screw.twist_matrix_from_angular_linear",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->twist_matrix_from_angular_linear == &inert::twist_matrix_from_angular_linear; }},
        slot_descriptor{"screw.twist_matrix_from_twist",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->twist_matrix_from_twist == &inert::twist_matrix_from_twist; }},
        slot_descriptor{"screw.screw_axis_from_angular_linear",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->screw_axis_from_angular_linear == &inert::screw_axis_from_angular_linear; }},
        slot_descriptor{"screw.screw_axis_from_point_direction_pitch", [](const void *value) -> bool
                        { return static_cast<const screw_ops *>(value)->screw_axis_from_point_direction_pitch == &inert::screw_axis_from_point_direction_pitch; }},
        slot_descriptor{"screw.matrix_exponential_so3",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_exponential_so3 == &inert::matrix_exponential_so3; }},
        slot_descriptor{"screw.matrix_exponential_se3",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_exponential_se3 == &inert::matrix_exponential_se3; }},
        slot_descriptor{"screw.matrix_exponential_screw",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_exponential_screw == &inert::matrix_exponential_screw; }},
        slot_descriptor{"screw.matrix_logarithm_so3",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_logarithm_so3 == &inert::matrix_logarithm_so3; }},
        slot_descriptor{"screw.matrix_logarithm_se3_rp",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_logarithm_se3_rp == &inert::matrix_logarithm_se3_rp; }},
        slot_descriptor{"screw.matrix_logarithm_se3",
                        [](const void *value) -> bool { return static_cast<const screw_ops *>(value)->matrix_logarithm_se3 == &inert::matrix_logarithm_se3; }},
};

static_assert(frame_descriptors.size() == static_cast<std::size_t>(frame_slot::count));
static_assert(screw_descriptors.size() == static_cast<std::size_t>(screw_slot::count));

constexpr capability_descriptors<frame_ops> described_frames{"rigid_motion", frame_descriptors};
constexpr capability_descriptors<screw_ops> described_screws{"rigid_motion", screw_descriptors};

}

capability_view view_of(const frame_ops &ops)
{
    return capability_view::of(ops, described_frames);
}

capability_view view_of(const screw_ops &ops)
{
    return capability_view::of(ops, described_screws);
}

}
