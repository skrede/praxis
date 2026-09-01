#include "praxis/rigid_motion/frame_stencil.h"

#include <threepp/objects/Group.hpp>

#include <threepp/math/MathUtils.hpp>

#include <array>
#include <cstddef>
#include <utility>

namespace praxis::rigid_motion {

namespace {

// The renderer stores a transform column by column, and in single precision.
threepp::Matrix4 to_renderer_transform(const transform &tf)
{
    std::array<float, 16> rendered{};
    for(Eigen::Index column = 0; column < 4; ++column)
        for(Eigen::Index row = 0; row < 4; ++row)
            rendered[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(tf(row, column));

    return threepp::Matrix4(rendered);
}

bool marks_origin(const stencil_object &described)
{
    return described.body.shape == body_shape::none;
}

}

frame_stencil::frame_stencil(threepp::Scene &parent, std::vector<stencil_object> objects, frame_ops motions, fixed_frame anchored)
        : m_tree(0, motions)
        , m_fixed(std::move(anchored))
        , m_scene(parent)
        , m_objects()
        , m_root(threepp::Group::create())
        , m_anchor()
        , m_marked()
        , m_marker()
        , m_selecting()
{
    m_root->rotation.x = -threepp::math::PI / 2.f;
    raise_marker();

    if(!m_fixed.name.empty())
    {
        m_anchor          = threepp::Group::create();
        m_anchor->name    = m_fixed.name;
        m_anchor->visible = m_fixed.axes.shown;
        m_anchor->add(make_axes(m_fixed.axes, true));
        m_root->add(m_anchor);
    }

    m_objects.reserve(objects.size());
    for(stencil_object &described : objects)
        add(std::move(described));
}

expected<void, refusal> frame_stencil::initialize()
{
    m_scene.add(m_root);
    m_scene.add(m_marker);

    return {};
}

void frame_stencil::tear_down()
{
    m_scene.remove(*m_root);
    m_scene.remove(*m_marker);
}

void frame_stencil::render() const
{
    for(std::size_t index = 0; index < m_objects.size(); ++index)
    {
        const threepp::Matrix4 put = to_renderer_transform(m_tree.world_pose(index));

        m_objects[index].node->position.setFromMatrixPosition(put);
        m_objects[index].node->quaternion.setFromRotationMatrix(put);
    }

    draw_mark();
}

std::size_t frame_stencil::count() const
{
    return m_objects.size();
}

bool frame_stencil::axes_shown(std::size_t index) const
{
    return index < m_objects.size() && m_objects[index].described.axes.shown;
}

const transform &frame_stencil::pose(std::size_t index) const
{
    return m_tree.pose(index);
}

std::string_view frame_stencil::name_of(std::size_t index) const
{
    if(index >= m_objects.size())
        return {};

    return m_objects[index].described.name;
}

transform frame_stencil::world_pose(std::size_t index) const
{
    return m_tree.world_pose(index);
}

std::optional<std::size_t> frame_stencil::parent_of(std::size_t index) const
{
    return m_tree.parent_of(index);
}

void frame_stencil::set_pose(std::size_t index, const transform &tf)
{
    m_tree.set_pose(index, tf);
}

void frame_stencil::set_body(std::size_t index, object_body chosen)
{
    if(index >= m_objects.size())
        return;

    placed &object        = m_objects[index];
    const bool marked     = marks_origin(object.described);
    object.described.body = std::move(chosen);

    attach_body(object);
    if(marked != marks_origin(object.described))
        attach_axes(object);
}

void frame_stencil::set_axes_shown(std::size_t index, bool shown)
{
    if(index >= m_objects.size())
        return;

    m_objects[index].described.axes.shown = shown;
    m_objects[index].axes->visible        = shown;
}

expected<void, refusal> frame_stencil::set_parent(std::size_t child, std::optional<std::size_t> parent)
{
    return m_tree.set_parent(child, parent);
}

std::size_t frame_stencil::add(stencil_object described)
{
    placed &object    = m_objects.emplace_back(placed{std::move(described), threepp::Group::create(), nullptr, nullptr});
    object.node->name = object.described.name;

    attach_axes(object);
    attach_body(object);
    m_root->add(object.node);

    return m_tree.add();
}

expected<void, refusal> frame_stencil::remove(std::size_t index)
{
    const expected<void, refusal> accepted = m_tree.remove(index);
    if(accepted)
        withdraw(index);

    return accepted;
}

// The axes node hangs whether or not it is drawn, so what the stencil put in the scene is a property
// of the object set rather than of what is currently visible.
void frame_stencil::attach_axes(placed &object)
{
    if(object.axes)
        object.node->remove(*object.axes);

    object.axes          = make_axes(object.described.axes, marks_origin(object.described));
    object.axes->visible = object.described.axes.shown;
    object.node->add(object.axes);
}

void frame_stencil::attach_body(placed &object)
{
    if(object.body)
        object.node->remove(*object.body);
    object.body = make_body(object.described.body);
    if(object.body)
        object.node->add(object.body);
}

// Objects hang flat under the one root and the parent relation is composed into the poses, so taking
// an object out is taking its own node out and no survivor needs re-parenting.
void frame_stencil::withdraw(std::size_t index)
{
    m_root->remove(*m_objects[index].node);
    m_objects.erase(m_objects.begin() + static_cast<std::ptrdiff_t>(index));
}

}
