#include "placement_reading.h"

#include "praxis/manipulator/model_placement.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"

#include <array>
#include <memory>
#include <cstddef>

namespace praxis::manipulator {

namespace {

// The renderer stores a transform column by column, and in single precision. Only the rotation
// block is read back from it: the object's position and scale are set on the node itself.
threepp::Matrix4 to_renderer_rotation(const rotation &r)
{
    std::array<float, 16> rendered{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f};
    for(Eigen::Index column = 0; column < 3; ++column)
        for(Eigen::Index row = 0; row < 3; ++row)
            rendered[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(r(row, column));

    return threepp::Matrix4(rendered);
}

void place_object(threepp::Object3D &object, const rigid_motion::frame_ops &frames, const world_object_window::settings &state, placement_doubts &doubts)
{
    const rotation orientation = read_rotation(frames, state.gfx_euler_zyx_degrees, axis_order::zyx, doubts);

    object.scale    = threepp::Vector3(state.gfx_scale.x(), state.gfx_scale.y(), state.gfx_scale.z());
    object.position = threepp::Vector3(state.gfx_offset.x(), state.gfx_offset.y(), state.gfx_offset.z());
    object.setRotationFromMatrix(to_renderer_rotation(orientation));
}

}

void note_reading(const rigid_motion::frame_ops &frames, rigid_motion::frame_slot slot, bool handed_finite, bool answered_finite, placement_doubts &doubts)
{
    if(holds_default(rigid_motion::view_of(frames), static_cast<std::size_t>(slot)))
        doubts.defaulted.set(slot);
    else if(handed_finite && !answered_finite)
        doubts.not_finite.set(slot);
}

rotation read_rotation(const rigid_motion::frame_ops &frames, const Eigen::Vector3f &euler_degrees, axis_order order, placement_doubts &doubts)
{
    const Eigen::Vector3d angles = euler_degrees.cast<double>() * radians_per_degree;
    const rotation read          = frames.rotation_matrix_from_euler(angles, order);
    note_reading(frames, rigid_motion::frame_slot::rotation_matrix_from_euler, angles.allFinite(), read.allFinite(), doubts);

    return read;
}

placement_doubts place_world_object(loadable_robot_stencil &on, const rigid_motion::frame_ops &frames, const world_object_window::settings &state)
{
    placement_doubts doubts;
    const std::shared_ptr<threepp::Object3D> object = on.world_object();
    if(object == nullptr)
        return doubts;

    if(state.active)
        place_object(*object, frames, state, doubts);
    else
        on.clear_world_object();

    return doubts;
}

void world_object_window::assign_gfx_transform()
{
    if(m_world_object == nullptr)
        return;

    placement_doubts unreported;
    place_object(*m_world_object, m_frame, state(), unreported);
}

}
