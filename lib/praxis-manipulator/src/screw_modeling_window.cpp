#include "inert_screw_report.h"

#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/rigid_motion/angles.h"

#include <spdlog/spdlog.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

// A screw whose angular part names no axis translates, so its numbers are an angular and a linear
// part; anything else has an axis, a point on it and a pitch. The boundary is the one the drawing
// reads the same screw at, so a row and the line it is drawn as never disagree about which it is.
screw_modeling_window::parameterization typed_as(const screw_axis &screw)
{
    return screw.head<3>().norm() <= angular_epsilon ? screw_modeling_window::parameterization::angular_linear : screw_modeling_window::parameterization::point_direction_pitch;
}

}

// A turning joint's `(z, 0)` and a translating joint's `(0, z)` are both what the published
// construction answers for that axis, so neither is written out here as a literal.
screw_axis screw_modeling_window::opening_screw(const rigid_motion::screw_ops &turning, const screw_axis &derived)
{
    if(typed_as(derived) == parameterization::angular_linear)
        return turning.screw_axis_from_angular_linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());

    const expected<screw_axis, refusal> built = turning.screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0);

    return built ? built.value() : screw_axis::Zero();
}

screw_modeling_window::settings::settings(transform chosen_home, std::vector<screw_axis> chosen_screws)
        : home(std::move(chosen_home))
        , screws(std::move(chosen_screws))
{
}

screw_modeling_window::controls::controls()
        : home(true)
        , reset(true)
{
}

screw_modeling_window::screw_modeling_window(std::string name, loadable_robot_stencil &target, arm_reader seen, const rigid_motion::screw_ops &turning,
                                             const rigid_motion::frame_ops &framing, const forward_kinematics_ops &solving, screw_chain derived)
        : screw_modeling_window(std::move(name), target, std::move(seen), turning, framing, solving, std::move(derived), controls(), settings{}, edit_route(), save_route())
{
}

screw_modeling_window::screw_modeling_window(std::string name, loadable_robot_stencil &target, arm_reader seen, const rigid_motion::screw_ops &turning,
                                             const rigid_motion::frame_ops &framing, const forward_kinematics_ops &solving, screw_chain derived, const controls &offered,
                                             const settings &state, edit_route edits, save_route save, std::string at)
        : imgui_window(std::move(name))
        , m_home(state.home)
        , m_seen(std::move(seen))
        , m_controls(offered)
        , m_derived(std::move(derived))
        , m_selected(0u)
        , m_settings_at(std::move(at))
        , m_kinematics(solving)
        , m_home_position(Eigen::Vector3f::Zero())
        , m_frame(framing)
        , m_screw(turning)
        , m_unbound(false)
        , m_screws(state.screws)
        , m_home_euler_degrees(Eigen::Vector3f::Zero())
        , m_stencil(target)
        , m_edits_cb(std::move(edits))
        , m_save_cb(std::move(save))
{
    seed(state);
}

// The table is as long as the derived chain, because that chain is what the rendered arm is posed
// from and what carries the limits a solve reads. A row nobody supplied opens degenerate.
void screw_modeling_window::seed(const settings &opened)
{
    const rotation held = m_home.block<3, 3>(0, 0);

    if(opened.screws.size() > m_derived.joint_count())
        spdlog::error("praxis: '{}' was supplied {} screws and the chain it opens against has {}, so the rest are dropped", display_name(), opened.screws.size(),
                      m_derived.joint_count());

    m_home_position      = m_home.block<3, 1>(0, 3).cast<float>();
    m_home_euler_degrees = (m_frame.euler_from_rotation_matrix(held, home_axis_order) * degrees_per_radian).cast<float>();

    m_selected = 0u;
    m_entries.clear();
    m_screws.resize(m_derived.joint_count());
    for(std::size_t joint = 0u; joint < m_screws.size(); ++joint)
    {
        if(joint >= opened.screws.size())
            m_screws[joint] = opening_screw(m_screw, m_derived.space_screws[joint]);

        m_entries.push_back("Joint " + std::to_string(joint + 1u));
        m_rows.emplace_back(typed_as(m_screws[joint]));
        m_rows.back().show(m_screws[joint]);
    }
}

screw_modeling_window::settings screw_modeling_window::state() const
{
    return settings{m_home, m_screws};
}

void screw_modeling_window::initialize()
{
    push();
    tell_selection();
}

// A chain of no joints has no joint to tell apart. Beyond that the drawing answers an index it
// carries no screw for by name, so the refusal it says is the whole answer and none is said twice.
void screw_modeling_window::tell_selection()
{
    if(m_entries.empty())
        return;

    static_cast<void>(m_stencil.set_selected_joint(m_selected));
}

std::vector<config::edit> screw_modeling_window::settings_edits(const config::document &carried) const
{
    return m_edits_cb ? m_edits_cb(carried, m_settings_at, state()) : std::vector<config::edit>();
}

void screw_modeling_window::assemble_home()
{
    const Eigen::Vector3d taken = m_home_euler_degrees.cast<double>() * radians_per_degree;

    m_home = m_frame.transformation_matrix_from_rotation_position(m_frame.rotation_matrix_from_euler(taken, home_axis_order), m_home_position.cast<double>());
    push();
}

// The whole table reaches the drawing on every change, so a wrong screw is visible against the link
// it claims to describe from the moment it is typed. A length the rendered arm cannot be drawn
// against is the derived chain's own, so it is reported rather than swallowed.
void screw_modeling_window::push()
{
    if(inert_and_reported(m_screw, m_stencil.inert_screw_slots(), rigid_motion::screw_slot::screw_axis_from_angular_linear,
                          "'" + display_name() + "' composes no chain from the rows it holds", m_unbound))
        return;

    if(!m_stencil.set_joint_screws(m_home, m_screws))
        spdlog::error("praxis: '{}' holds {} screws and the arm they are drawn against does not take that many", display_name(), m_screws.size());
}

void screw_modeling_window::reset()
{
    m_home = transform::Identity();
    m_rows.clear();
    m_screws.clear();
    seed(settings{});
    push();
    tell_selection();
}

void screw_modeling_window::rebuild_row(std::size_t joint)
{
    const row &shown = m_rows[joint];
    if(shown.typed == parameterization::angular_linear)
    {
        m_screws[joint] = m_screw.screw_axis_from_angular_linear(shown.angular.cast<double>(), shown.linear.cast<double>());

        return push();
    }

    const expected<screw_axis, refusal> built =
            m_screw.screw_axis_from_point_direction_pitch(shown.point.cast<double>(), shown.direction.cast<double>(), static_cast<double>(shown.pitch));
    if(!built)
        return refuse(joint);

    m_screws[joint] = built.value();
    push();
}

void screw_modeling_window::refuse(std::size_t joint)
{
    spdlog::error("praxis: 'rigid_motion.screw.screw_axis_from_point_direction_pitch' named no axis for joint {} of '{}', so the screw it carried is kept", joint + 1u, display_name());
    canonicalize(joint);
}

void screw_modeling_window::canonicalize(std::size_t joint)
{
    m_rows[joint].show(m_screws[joint]);
}

}
