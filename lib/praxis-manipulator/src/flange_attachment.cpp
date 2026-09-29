#include "robot/joint_decoration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/axes.h"

#include <threepp/math/Box3.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <memory>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr double marker_axis_thickness_fraction = 0.10;

// The extent stood in for where the arm has none of its own, in metres.
constexpr double bare_arm_extent = 1.0;

std::size_t slot_of(flange_attachment which)
{
    return static_cast<std::size_t>(which);
}

bool marks_a_frame(flange_attachment which)
{
    return which == flange_attachment::frame_marker || which == flange_attachment::tool_frame_marker;
}

threepp::Matrix4 carried_by(threepp::Matrix4 base, const threepp::Matrix4 &offset)
{
    base.multiply(offset);

    return base;
}

// The supplied chain's poses are in the model's root-link frame, which `root` carries into the scene.
threepp::Matrix4 placed_at(flange_attachment which, const threepp::Matrix4 &offset, const threepp::Matrix4 &flange, const std::shared_ptr<const arm_snapshot> &seen,
                           const std::optional<chain_end> &supplied, const threepp::Matrix4 &root)
{
    if(supplied && which == flange_attachment::frame_marker)
        return carried_by(carried_by(root, to_renderer_transform(supplied->flange)), offset);
    if(supplied && which == flange_attachment::tool_frame_marker)
        return carried_by(root, to_renderer_transform(supplied->tool));
    if(which == flange_attachment::tool_frame_marker && seen != nullptr)
        return carried_by(flange, to_renderer_transform(seen->tool_offset));

    return carried_by(flange, offset);
}

}

std::shared_ptr<threepp::Object3D> make_flange_marker(threepp::Object3D &arm)
{
    return make_flange_marker(arm, opening_marker_extent_fraction);
}

std::shared_ptr<threepp::Object3D> make_flange_marker(threepp::Object3D &arm, double of_the_arms_extent)
{
    threepp::Box3 around;
    around.setFromObject(arm);

    const double extent = around.isEmpty() ? bare_arm_extent : static_cast<double>(around.getSize().length());

    rigid_motion::axes_settings chosen;
    chosen.axis_length    = extent * (of_the_arms_extent > 0.0 ? of_the_arms_extent : opening_marker_extent_fraction);
    chosen.axis_thickness = chosen.axis_length * marker_axis_thickness_fraction;

    return rigid_motion::make_axes(chosen, true);
}

void loadable_robot_stencil::set_flange_attachment(flange_attachment which, std::shared_ptr<threepp::Object3D> attached)
{
    set_flange_attachment(which, std::move(attached), threepp::Matrix4());
}

void loadable_robot_stencil::set_flange_attachment(flange_attachment which, std::shared_ptr<threepp::Object3D> attached, threepp::Matrix4 offset)
{
    clear_flange_attachment(which);

    carried &held = m_attached[slot_of(which)];
    held.object   = std::move(attached);
    held.offset   = offset;
    m_scene.add(held.object);
}

void loadable_robot_stencil::set_flange_attachment_offset(flange_attachment which, threepp::Matrix4 offset)
{
    m_attached[slot_of(which)].offset = offset;
}

void loadable_robot_stencil::clear_flange_attachment(flange_attachment which)
{
    carried &held = m_attached[slot_of(which)];
    if(held.object == nullptr)
        return;

    m_scene.remove(*held.object);
    held.object.reset();
    held.offset.identity();
}

void loadable_robot_stencil::set_flange_marker_policy(flange_marker_policy under)
{
    m_marker_policy = under;
}

flange_marker_policy loadable_robot_stencil::flange_marker_policy_held() const
{
    return m_marker_policy;
}

void loadable_robot_stencil::set_flange_marker_shown(bool shown)
{
    m_marker_shown = shown;
}

void loadable_robot_stencil::set_tool_marker_shown(bool shown)
{
    m_tool_marker_shown = shown;
}

expected<void, refusal> loadable_robot_stencil::set_marker_scale(double of_the_built_size)
{
    if(of_the_built_size <= 0.0)
        return unexpected(refusal::unsupported_input);

    m_marker_scale = of_the_built_size;

    return {};
}

double loadable_robot_stencil::marker_scale() const
{
    return m_marker_scale;
}

std::shared_ptr<threepp::Object3D> loadable_robot_stencil::attached_at(flange_attachment which) const
{
    return m_attached[slot_of(which)].object;
}

void loadable_robot_stencil::detach_flange_attachments()
{
    for(const carried &held : m_attached)
        if(held.object != nullptr)
            m_scene.remove(*held.object);
}

// Each attachment stands at the flange composed with its offset, the tool frame's marker at the tool
// offset the arm published. While a supplied chain is held the two frame markers stand where it ends.
void loadable_robot_stencil::place_flange_attachments() const
{
    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    const threepp::Matrix4 flange                  = m_robot->getEndEffectorTransform();
    const std::optional<chain_end> supplied        = supplied_marker_poses(seen);
    const threepp::Matrix4 root                    = root_frame();
    for(std::size_t slot = 0; slot < flange_attachment_count; ++slot)
    {
        const carried &held           = m_attached[slot];
        const flange_attachment which = static_cast<flange_attachment>(slot);
        if(held.object == nullptr)
            continue;

        const threepp::Matrix4 at = placed_at(which, held.offset, flange, seen, supplied, root);
        held.object->position.setFromMatrixPosition(at);
        held.object->quaternion.setFromRotationMatrix(at);
        if(marks_a_frame(which))
        {
            const auto worn = static_cast<float>(m_marker_scale);
            held.object->scale.set(worn, worn, worn);
        }
    }

    show_flange_markers();
}

void loadable_robot_stencil::show_flange_markers() const
{
    const carried &at_the_flange = m_attached[slot_of(flange_attachment::frame_marker)];
    if(at_the_flange.object != nullptr)
    {
        const bool occupied           = m_attached[slot_of(flange_attachment::tool)].object != nullptr;
        const bool withheld           = m_marker_policy == flange_marker_policy::yields && occupied;
        at_the_flange.object->visible = m_marker_shown && !withheld;
    }

    const carried &at_the_tool = m_attached[slot_of(flange_attachment::tool_frame_marker)];
    if(at_the_tool.object != nullptr)
        at_the_tool.object->visible = m_tool_marker_shown;
}

}
