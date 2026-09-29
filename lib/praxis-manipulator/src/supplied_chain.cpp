#include "inert_screw_report.h"

#include "robot/chain_placement.h"
#include "robot/joint_decoration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/math/Matrix4.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/materials/Material.hpp>

#include <format>
#include <memory>
#include <string>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr double described_marker_extent_fraction = 0.10;
constexpr float described_marker_opacity          = 0.5f;

withheld_chain counted_apart(std::size_t screws, std::size_t joints)
{
    return withheld_chain{withheld_cause::joint_count, std::format("The supplied chain is not folded: it holds {} screws and the arm has {} joints.", screws, joints)};
}

withheld_chain left_unbound(const rigid_motion::screw_ops &described, rigid_motion::screw_slot slot)
{
    return withheld_chain{withheld_cause::unbound_slot, std::format("The supplied chain is not folded: '{}' holds its default.", screw_slot_name(described, slot))};
}

withheld_chain refused_at(std::size_t joint)
{
    return withheld_chain{withheld_cause::refused, std::format("The supplied chain is not folded: joint {}'s screw was refused.", joint + 1u)};
}

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
    m_supplied = false;
    rebuild_decoration();

    return {};
}

void loadable_robot_stencil::clear_joint_screws()
{
    m_screws.clear();
    m_supplied = false;
    rebuild_decoration();
}

// A count the rendered arm cannot take is refused as set_joint_screws refuses it, and the chain is
// still supplied so that it is withheld naming both counts.
expected<void, refusal> loadable_robot_stencil::supply_joint_screws(const transform &home, std::span<const screw_axis> space_screws)
{
    m_supplied_count = space_screws.size();
    m_unbuilt.reset();
    const expected<void, refusal> told = set_joint_screws(home, space_screws);
    m_supplied                         = true;

    return told;
}

void loadable_robot_stencil::supply_unbuilt_chain(rigid_motion::screw_slot unbound)
{
    m_supplied = true;
    m_unbuilt  = unbound;
}

bool loadable_robot_stencil::holds_supplied_chain() const
{
    return m_supplied;
}

void loadable_robot_stencil::set_described_marker_shown(bool shown)
{
    m_described_marker_shown = shown;
}

std::shared_ptr<threepp::Object3D> make_described_flange_marker(threepp::Object3D &arm)
{
    std::shared_ptr<threepp::Object3D> built = make_flange_marker(arm, described_marker_extent_fraction);
    built->traverseType<threepp::Mesh>(
            [](threepp::Mesh &part)
            {
                part.material()->transparent = true;
                part.material()->opacity     = described_marker_opacity;
            });

    return built;
}

expected<chain_end, withheld_chain> loadable_robot_stencil::supplied_chain_end(const arm_snapshot &seen) const
{
    const std::size_t rendered = m_robot->numDOF();
    if(m_unbuilt)
        return unexpected(left_unbound(m_screw, *m_unbuilt));
    if(m_supplied_count != rendered)
        return unexpected(counted_apart(m_supplied_count, rendered));
    if(m_inert.contains(rigid_motion::screw_slot::matrix_exponential_screw))
        return unexpected(left_unbound(m_screw, rigid_motion::screw_slot::matrix_exponential_screw));

    const expected<chain_fold, refused_joint> folded = fold_joint_origins(m_home, m_screws, seen.joints, m_screw);
    if(!folded)
        return unexpected(refused_at(folded.error().joint));

    return chain_end{folded->reached, transform(folded->reached * seen.tool_offset)};
}

// The frame getEndEffectorTransform() answers the flange in. It is composed from the robot node's
// own placement because the node's world matrix is refreshed only after the frame is drawn.
threepp::Matrix4 loadable_robot_stencil::root_frame() const
{
    threepp::Matrix4 placed;
    placed.compose(m_robot->position, m_robot->quaternion, m_robot->scale);

    threepp::Matrix4 root(*m_scene.matrixWorld);
    root.multiply(placed);

    return root;
}

std::optional<chain_end> loadable_robot_stencil::supplied_marker_poses(const std::shared_ptr<const arm_snapshot> &seen) const
{
    if(!m_supplied || seen == nullptr)
        return std::nullopt;

    const expected<chain_end, withheld_chain> end = supplied_chain_end(*seen);
    if(!end)
        return chain_end{transform::Identity(), transform::Identity()};

    return *end;
}

}
