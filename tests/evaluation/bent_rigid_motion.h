#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_RIGID_MOTION_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_RIGID_MOTION_H

#include "bent_frame.h"
#include "bent_screw.h"
#include "bend_visibility.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/generation.h"

#include "praxis/extension/coverage.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <utility>
#include <string_view>

// Two fixtures over the same aggregate. The first is seven slots, each answering the reference
// displaced by `bend_by` in the one quantity its own residual measures, so a residual read off any
// of them is the displacement itself: a matrix entry moved, an angle turned, an origin displaced, an
// axis component moved, a logarithm's angle turned, a logarithm's linear half moved. A displacement
// is set through `bend_by` because a slot is a plain function pointer and carries nothing of its own.
// The second, at the foot of the file, reaches one bend per slot by the index the report lists it at.
namespace praxis::fixture {

enum class bent : std::size_t
{
    element_wise,
    geodesic,
    pose_radians,
    pose_metres,
    axis_up_to_sign,
    log_radians,
    log_metres,
    count,
};

inline std::array<double, static_cast<std::size_t>(bent::count)> bend_by{};

inline double bent_by(bent which)
{
    return bend_by[static_cast<std::size_t>(which)];
}

inline matrix4 moved_entry(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    matrix4 m = rigid_motion::twist_matrix_from_angular_linear(w, v);
    m(0, 1) += bent_by(bent::element_wise);

    return m;
}

inline rotation turned_further(double radians)
{
    return rigid_motion::rotate_z(radians + bent_by(bent::geodesic));
}

inline transform turned_pose(const rotation &r)
{
    const rotation further(Eigen::AngleAxisd(bent_by(bent::pose_radians), Eigen::Vector3d::UnitX()).toRotationMatrix());

    return rigid_motion::transformation_matrix_from_rotation(rotation(further * r));
}

inline transform displaced_pose(const Eigen::Vector3d &p)
{
    return rigid_motion::transformation_matrix_from_position(Eigen::Vector3d(p + bent_by(bent::pose_metres) * Eigen::Vector3d::UnitX()));
}

inline screw_axis moved_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    screw_axis axis = rigid_motion::screw_axis_from_angular_linear(w, v);
    axis[0] += bent_by(bent::axis_up_to_sign);

    return axis;
}

inline expected<std::pair<Eigen::Vector3d, double>, refusal> turned_logarithm(const rotation &r)
{
    const expected<std::pair<Eigen::Vector3d, double>, refusal> read = rigid_motion::matrix_logarithm_so3(r);
    if(!read)
        return read;

    return std::pair<Eigen::Vector3d, double>{read.value().first, read.value().second + bent_by(bent::log_radians)};
}

inline expected<std::pair<screw_axis, double>, refusal> displaced_logarithm(const transform &tf)
{
    const expected<std::pair<screw_axis, double>, refusal> read = rigid_motion::matrix_logarithm_se3(tf);
    if(!read)
        return read;

    screw_axis axis = read.value().first;
    axis[3] += bent_by(bent::log_metres);

    return std::pair<screw_axis, double>{axis, read.value().second};
}

inline rigid_motion::capabilities bent_everywhere()
{
    rigid_motion::capabilities spatial                = rigid_motion::baseline();
    spatial.frame.rotate_z                            = &turned_further;
    spatial.frame.transformation_matrix_from_position = &displaced_pose;
    spatial.frame.transformation_matrix_from_rotation = &turned_pose;
    spatial.screw.twist_matrix_from_angular_linear    = &moved_entry;
    spatial.screw.screw_axis_from_angular_linear      = &moved_axis;
    spatial.screw.matrix_logarithm_so3                = &turned_logarithm;
    spatial.screw.matrix_logarithm_se3                = &displaced_logarithm;

    return spatial;
}

inline bool is_bent(std::string_view slot)
{
    return slot == "frame.rotate_z" || slot == "frame.transformation_matrix_from_position" || slot == "frame.transformation_matrix_from_rotation" ||
            slot == "screw.twist_matrix_from_angular_linear" || slot == "screw.screw_axis_from_angular_linear" || slot == "screw.matrix_logarithm_so3" ||
            slot == "screw.matrix_logarithm_se3";
}

// The index is the order the report lists slots in: the twelve frame slots in enumerator order, then
// the seventeen screw slots. All three are total, so an index past the last slot answers the
// reference unchanged, the empty name, and no visible difference.
inline rigid_motion::capabilities bent_at(std::size_t index)
{
    rigid_motion::capabilities spatial = rigid_motion::baseline();

    if(index < frame_bends.size())
        frame_bends[index](spatial);
    else if(index - frame_bends.size() < screw_bends.size())
        screw_bends[index - frame_bends.size()](spatial);

    return spatial;
}

inline std::string_view bent_slot_name(std::size_t index)
{
    const rigid_motion::frame_ops frame{};
    const rigid_motion::screw_ops screw{};

    if(index < frame_bends.size())
        return slot_name(rigid_motion::view_of(frame), index);

    return slot_name(rigid_motion::view_of(screw), index - frame_bends.size());
}

inline bool bend_is_visible(std::size_t index, const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn)
{
    if(index < frame_bend_probes.size())
        return frame_bend_probes[index](held, bent, drawn);

    return index - frame_bend_probes.size() < screw_bend_probes.size() && screw_bend_probes[index - frame_bend_probes.size()](held, bent, drawn);
}

}

#endif
