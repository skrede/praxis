#include "praxis/rigid_motion/frame_stencil.h"

#include <threepp/math/Color.hpp>

#include <threepp/helpers/Box3Helper.hpp>

#include <cstddef>
#include <utility>

namespace praxis::rigid_motion {

namespace {

// The tone the mark's lines are drawn in.
const threepp::Color mark_tone = threepp::Color::purple;

// How far the mark stands off what it marks, in metres, in every direction.
constexpr float mark_standoff = 0.030f;

// The mark tells one object from the others, so a set smaller than this has nothing to tell one
// apart from and draws none.
constexpr std::size_t least_marked_set = 2;

void mark_nothing(threepp::Box3 &box, threepp::Object3D &marker)
{
    box.makeEmpty();
    marker.visible = false;
}

}

void frame_stencil::raise_marker()
{
    auto drawn     = threepp::Box3Helper::create(m_marked, mark_tone);
    drawn->name    = "mark";
    drawn->visible = false;

    m_marker = std::move(drawn);
}

void frame_stencil::follow_selection(std::function<std::size_t()> selecting)
{
    m_selecting = std::move(selecting);
}

// The marker reads the box on every world-matrix update the renderer gives it, so moving the mark is
// writing this box and nothing else: the node is raised once and is never re-made, re-pointed or
// re-parented. An empty box leaves that update with a placement it never composed, so the marker is
// visible exactly when the box is not empty.
void frame_stencil::draw_mark() const
{
    if(m_objects.size() < least_marked_set)
    {
        mark_nothing(m_marked, *m_marker);
        return;
    }

    const std::size_t marked = m_selecting ? m_selecting() : m_objects.size();
    if(marked >= m_objects.size())
    {
        mark_nothing(m_marked, *m_marker);
        return;
    }

    // The mark covers the object node's whole extent, body or none, and the parent chain carries the
    // root's turn onto the renderer's world.
    threepp::Object3D &node = *m_objects[marked].node;
    node.updateWorldMatrix(true, false);
    m_marked.setFromObject(node);

    m_marker->visible = !m_marked.isEmpty();
    if(m_marker->visible)
        m_marked.expandByScalar(mark_standoff);
}

}
