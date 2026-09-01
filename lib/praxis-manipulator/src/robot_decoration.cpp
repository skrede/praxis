#include "robot/chain_figure.h"
#include "robot/chain_placement.h"
#include "robot/joint_decoration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/objects/Group.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <memory>
#include <cstddef>

namespace praxis::manipulator {

std::string loadable_robot_stencil::joint_axis_name(std::size_t joint)
{
    return "Joint screw axis " + std::to_string(joint + 1);
}

std::string loadable_robot_stencil::chain_name()
{
    return "Joint chain";
}

std::string loadable_robot_stencil::chain_segment_name(std::size_t segment)
{
    return "Joint chain segment " + std::to_string(segment + 1);
}

std::string loadable_robot_stencil::joint_mark_name(std::size_t joint)
{
    return "Joint origin mark " + std::to_string(joint + 1);
}

expected<void, refusal> loadable_robot_stencil::set_joint_screws(const transform &home, std::span<const screw_axis> space_screws)
{
    const std::size_t rendered = m_robot->numDOF();
    if(space_screws.size() != rendered)
    {
        spdlog::error("praxis: the rendered arm has {} joints and the screws it was told name {}, so the axes cannot be drawn against it", rendered, space_screws.size());

        return unexpected(refusal::unsupported_input);
    }

    m_home = home;
    m_screws.assign(space_screws.begin(), space_screws.end());
    rebuild_decoration();

    return {};
}

void loadable_robot_stencil::clear_joint_screws()
{
    m_screws.clear();
    rebuild_decoration();
}

bool loadable_robot_stencil::holds_chain() const
{
    return m_chain != nullptr;
}

void loadable_robot_stencil::set_decoration_reach(double metres)
{
    m_reach = metres;
    rebuild_decoration();
}

double loadable_robot_stencil::decoration_reach() const
{
    return m_reach;
}

void loadable_robot_stencil::set_meshes_shown(bool shown)
{
    m_robot->visible = shown;
}

void loadable_robot_stencil::set_decoration_shown(bool shown)
{
    m_axes->visible = shown;
}

void loadable_robot_stencil::set_chain_shown(bool shown)
{
    m_figure->visible = shown;
}

expected<void, refusal> loadable_robot_stencil::set_selected_joint(std::size_t joint)
{
    if(joint >= m_screws.size())
    {
        spdlog::error("praxis: the drawing carries {} joints and the one it was told to tell apart is {}, so the selection is declined", m_screws.size(), joint + 1u);

        return unexpected(refusal::unsupported_input);
    }

    m_selected = joint;
    apply_selection();

    return {};
}

void loadable_robot_stencil::clear_selected_joint()
{
    m_selected.reset();
    apply_selection();
}

// The geometry is built where the screws arrive rather than where they are drawn: the placement runs
// every frame on the render strand, and a buffer allocated there is one allocated per frame.
void loadable_robot_stencil::rebuild_decoration()
{
    for(const std::shared_ptr<threepp::Object3D> &drawn : m_drawn)
        m_axes->remove(*drawn);
    m_drawn.clear();

    for(std::size_t joint = 0; joint < m_screws.size(); ++joint)
    {
        m_drawn.push_back(joint_axis_object(joint_axis_name(joint), m_reach, m_axis_tone));
        m_axes->add(m_drawn.back());
    }

    rebuild_chain();
    apply_selection();
}

// The chain runs through the frame's origin, one point per joint and the tool point, so its segment
// count and its mark count follow the joint count and nothing else; a reach or a configuration that
// changed leaves the objects already standing.
void loadable_robot_stencil::rebuild_chain()
{
    const std::size_t marks    = m_screws.size();
    const std::size_t segments = m_screws.empty() ? 0u : m_screws.size() + 1u;
    if(m_segments.size() == segments && m_marks.size() == marks)
        return;

    clear_chain();
    if(segments == 0u)
        return;

    m_chain       = threepp::Group::create();
    m_chain->name = chain_name();
    for(std::size_t at = 0; at < segments; ++at)
        m_chain->add(m_segments.emplace_back(chain_segment_object(chain_segment_name(at), m_chain_tone)));
    for(std::size_t joint = 0; joint < marks; ++joint)
        m_chain->add(m_marks.emplace_back(joint_mark_object(joint_mark_name(joint), m_chain_tone)));

    m_figure->add(m_chain);
}

void loadable_robot_stencil::clear_chain()
{
    if(m_chain != nullptr)
        m_figure->remove(*m_chain);
    m_chain.reset();
    m_segments.clear();
    m_marks.clear();
}

// Joint j owns the point at fold index j+1, the segment spanning fold indices j and j+1 and the axis
// line named for j, so one index reaches all three of a joint's drawn items.
void loadable_robot_stencil::apply_selection() const
{
    for(std::size_t joint = 0; joint < m_drawn.size(); ++joint)
        wear(*m_drawn[joint], m_selected == joint ? m_axis_told : m_axis_tone);

    for(std::size_t at = 0; at < m_segments.size(); ++at)
        wear(*m_segments[at], m_selected == at ? m_chain_told : m_chain_tone);

    for(std::size_t joint = 0; joint < m_marks.size(); ++joint)
        wear(*m_marks[joint], m_selected == joint ? m_chain_told : m_chain_tone);
}

void loadable_robot_stencil::place_joint_decoration() const
{
    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    if(seen == nullptr)
        return;

    if(decline_unbound_fold(m_drawn, m_chain.get(), m_screw, m_inert, m_reported_unbound))
        return;

    place_joint_axes(m_drawn, m_screws, seen->joints, m_screw);
    if(m_chain == nullptr)
        return;

    const expected<std::vector<Eigen::Vector3d>, refusal> folded = fold_joint_origins(m_home, m_screws, seen->joints, m_screw);
    m_chain->visible                                             = folded.has_value();
    if(folded)
        place_chain_figure(m_segments, m_marks, *folded);
}

}
