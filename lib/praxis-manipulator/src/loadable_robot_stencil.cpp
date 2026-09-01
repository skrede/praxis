#include "robot/chain_figure.h"
#include "robot/column_arrow.h"
#include "robot/ellipsoid_figure.h"
#include "robot/joint_decoration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/extension/held_handle.h"

#include "praxis/scheduler/ownership.h"

#include <threepp/objects/Group.hpp>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

std::vector<float> to_renderer(const joint_vector &positions)
{
    std::vector<float> values;
    values.reserve(static_cast<std::size_t>(positions.size()));
    for(Eigen::Index i = 0; i < positions.size(); ++i)
        values.push_back(static_cast<float>(positions[i]));

    return values;
}

// Drawn metres per unit semi-axis.
constexpr double opening_angular_scale = 0.04;
constexpr double opening_linear_scale  = 0.125;

// Dimensionless: a multiple of the block's own ellipsoid scale.
constexpr double opening_force_cap_ratio = 2.4;

// Drawn metres per unit of a Jacobian column's angular part and of its linear part.
constexpr double opening_angular_column_scale = 0.1;
constexpr double opening_linear_column_scale  = 0.6;

std::size_t block_of(jacobian_block which)
{
    return static_cast<std::size_t>(which);
}

std::vector<std::shared_ptr<threepp::Material>> body_ramp(bool wireframe)
{
    std::vector<std::shared_ptr<threepp::Material>> ramp;
    ramp.reserve(ellipsoid_ramp_steps());
    for(std::size_t step = 0; step < ellipsoid_ramp_steps(); ++step)
        ramp.push_back(ellipsoid_material(step, wireframe));

    return ramp;
}

std::vector<std::shared_ptr<threepp::Material>> line_ramp()
{
    std::vector<std::shared_ptr<threepp::Material>> ramp;
    ramp.reserve(ellipsoid_ramp_steps());
    for(std::size_t step = 0; step < ellipsoid_ramp_steps(); ++step)
        ramp.push_back(continuation_material(step));

    return ramp;
}

}

loadable_robot_stencil::loadable_robot_stencil(std::shared_ptr<threepp::Robot> robot_object, attached_models attached, threepp::Scene &parent, scheduler::strand render, arm_reader seen,
                                               rigid_motion::screw_ops screw, rigid_motion::screw_slot_set inert)
        : m_seen(std::move(seen))
        , m_scene(parent)
        , m_render(render)
        , m_screw(screw)
        , m_inert(inert)
        , m_reported_unbound(false)
        , m_reach(opening_axis_reach(robot_object.get()))
        , m_home(transform::Identity())
        , m_axis_tone(axis_material(false))
        , m_axis_told(axis_material(true))
        , m_chain_tone(chain_material(false))
        , m_chain_told(chain_material(true))
        , m_solution_tone(solution_material())
        , m_robot(std::move(robot_object))
        , m_axes(threepp::Group::create())
        , m_figure(threepp::Group::create())
        , m_paths(threepp::Group::create())
        , m_solutions(threepp::Group::create())
        , m_ellipsoid_groups{threepp::Group::create(), threepp::Group::create()}
        , m_columns(threepp::Group::create())
        , m_decoration(threepp::Group::create())
        , m_ellipsoid_scale{opening_angular_scale, opening_linear_scale}
        , m_column_scale{opening_angular_column_scale, opening_linear_column_scale}
        , m_force_cap_ratio(opening_force_cap_ratio)
        , m_force_capped(true)
        , m_frame(jacobian_frame::space)
        , m_view(ellipsoid_view::velocity)
        , m_ellipsoid_solid(body_ramp(false))
        , m_ellipsoid_wire(body_ramp(true))
        , m_continuation_tone(line_ramp())
        , m_column_tone{column_material(jacobian_block::angular), column_material(jacobian_block::linear)}
{
    held(m_robot, "the loadable robot stencil", "robot object").rotation.x = -threepp::math::PI / 2.f;
    m_decoration->rotation.x                                               = -threepp::math::PI / 2.f;

    // Seven subtrees under the one root, so that the switch over the axes, the switch over the
    // chain, the switch over any one path, the switch over the figures the stencil was told, the
    // switch over each of the two manipulability ellipsoids and the switch over the columns of the
    // Jacobian it is showing are each a write the others cannot reach.
    m_decoration->add(m_axes);
    m_decoration->add(m_figure);
    m_decoration->add(m_paths);
    m_decoration->add(m_solutions);
    m_decoration->add(m_ellipsoid_groups[block_of(jacobian_block::angular)]);
    m_decoration->add(m_ellipsoid_groups[block_of(jacobian_block::linear)]);
    m_decoration->add(m_columns);

    if(attached.tool != nullptr)
        set_flange_attachment(flange_attachment::tool, std::move(attached.tool));
    if(attached.world != nullptr)
        set_world_object(std::move(attached.world));
}

expected<void, refusal> loadable_robot_stencil::initialize()
{
    m_scene.add(m_robot);
    m_scene.add(m_decoration);

    return {};
}

void loadable_robot_stencil::tear_down()
{
    if(m_world_object)
        m_scene.remove(*m_world_object);
    detach_flange_attachments();
    m_scene.remove(*m_decoration);
    m_scene.remove(*m_robot);
}

void loadable_robot_stencil::set_world_object(std::shared_ptr<threepp::Object3D> world_object)
{
    clear_world_object();
    m_world_object = std::move(world_object);
    m_scene.add(m_world_object);
}

void loadable_robot_stencil::clear_world_object()
{
    if(m_world_object)
    {
        m_scene.remove(*m_world_object);
        m_world_object.reset();
    }
}

std::shared_ptr<threepp::Object3D> loadable_robot_stencil::world_object() const
{
    return m_world_object;
}

threepp::Robot &loadable_robot_stencil::robot()
{
    return *m_robot;
}

const threepp::Robot &loadable_robot_stencil::robot() const
{
    return *m_robot;
}

// The only place in the tree that writes the node's joint values, and the assertion above the write
// is what makes that checkable rather than promised.
void loadable_robot_stencil::apply_published() const
{
    scheduler::require_strand(m_render, "the rendered robot");

    const std::shared_ptr<const arm_snapshot> seen = m_seen.read();
    if(seen == nullptr)
        return;

    m_robot->setJointValues(to_renderer(seen->joints));
}

void loadable_robot_stencil::render() const
{
    apply_published();
    place_joint_decoration();
    place_flange_attachments();
    place_ellipsoids();
    place_jacobian_columns();
}

}
