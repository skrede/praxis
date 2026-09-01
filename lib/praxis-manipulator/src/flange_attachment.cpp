#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/axes.h"

#include <threepp/math/Box3.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <memory>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr double marker_axis_length_fraction    = 0.15;
constexpr double marker_axis_thickness_fraction = 0.10;

// The extent stood in for where the arm has none of its own, in metres.
constexpr double bare_arm_extent = 1.0;

std::size_t slot_of(flange_attachment which)
{
    return static_cast<std::size_t>(which);
}

}

std::shared_ptr<threepp::Object3D> make_flange_marker(threepp::Object3D &arm)
{
    threepp::Box3 around;
    around.setFromObject(arm);

    const double extent = around.isEmpty() ? bare_arm_extent : static_cast<double>(around.getSize().length());

    rigid_motion::axes_settings chosen;
    chosen.axis_length    = extent * marker_axis_length_fraction;
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

// The one rule every attachment is carried by: the flange's pose composed with the offset the
// attachment was installed under.
void loadable_robot_stencil::place_flange_attachments() const
{
    const threepp::Matrix4 flange = m_robot->getEndEffectorTransform();
    for(const carried &held : m_attached)
    {
        if(held.object == nullptr)
            continue;

        threepp::Matrix4 at(flange);
        at.multiply(held.offset);
        held.object->position.setFromMatrixPosition(at);
        held.object->quaternion.setFromRotationMatrix(at);
    }
}

}
