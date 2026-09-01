#include "praxis/rigid_motion/frame_tree.h"

#include <cstddef>
#include <optional>

namespace praxis::rigid_motion {

namespace {

const transform nowhere = transform::Identity();

}

frame_tree::frame_tree(std::size_t frames, frame_ops)
        : m_links(frames, link{transform::Identity(), std::nullopt})
{
}

std::size_t frame_tree::count() const
{
    return m_links.size();
}

const transform &frame_tree::pose(std::size_t index) const
{
    if(index >= m_links.size())
        return nowhere;

    return m_links[index].placement;
}

transform frame_tree::world_pose(std::size_t index) const
{
    if(index >= m_links.size())
        return nowhere;

    return root_pose(index);
}

std::optional<std::size_t> frame_tree::parent_of(std::size_t index) const
{
    if(index >= m_links.size())
        return std::nullopt;

    return m_links[index].parent;
}

void frame_tree::set_pose(std::size_t index, const transform &tf)
{
    if(index >= m_links.size())
        return;

    m_links[index].placement = tf;
}

expected<void, refusal> frame_tree::set_parent(std::size_t child, std::optional<std::size_t> parent)
{
    if(child >= m_links.size())
        return unexpected(refusal::unsupported_input);
    if(parent && (*parent >= m_links.size() || *parent == child || descends_from(*parent, child)))
        return unexpected(refusal::unsupported_input);

    m_links[child].parent = parent;

    return {};
}

std::size_t frame_tree::add()
{
    m_links.push_back(link{transform::Identity(), std::nullopt});

    return m_links.size() - 1;
}

expected<void, refusal> frame_tree::remove(std::size_t index)
{
    if(index >= m_links.size())
        return unexpected(refusal::unsupported_input);
    if(expressed_in(index))
        return unexpected(refusal::no_solution);

    m_links.erase(m_links.begin() + static_cast<std::ptrdiff_t>(index));
    carry_indices_across(index);

    return {};
}

transform frame_tree::root_pose(std::size_t index) const
{
    transform composed               = m_links[index].placement;
    std::optional<std::size_t> above = m_links[index].parent;
    for(std::size_t step = 0; step < m_links.size() && above; ++step)
    {
        composed = m_links[*above].placement * composed;
        above    = m_links[*above].parent;
    }

    return composed;
}

bool frame_tree::descends_from(std::size_t node, std::size_t ancestor) const
{
    std::optional<std::size_t> above = m_links[node].parent;
    for(std::size_t step = 0; step < m_links.size() && above; ++step)
    {
        if(*above == ancestor)
            return true;
        above = m_links[*above].parent;
    }

    return false;
}

bool frame_tree::expressed_in(std::size_t index) const
{
    for(const link &held : m_links)
        if(held.parent == index)
            return true;

    return false;
}

void frame_tree::carry_indices_across(std::size_t removed)
{
    for(link &held : m_links)
        if(held.parent && *held.parent > removed)
            --*held.parent;
}

}
