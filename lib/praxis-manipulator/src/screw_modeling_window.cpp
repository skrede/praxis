#include "inert_screw_report.h"
#include "screw_modeling_padding.h"

#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/rigid_motion/angles.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::manipulator {

screw_modeling_window::settings::settings(transform chosen_home, std::vector<supplied_screw> chosen_screws)
        : home(std::move(chosen_home))
        , screws(std::move(chosen_screws))
{
}

screw_modeling_window::settings::settings(transform chosen_home, const std::vector<screw_axis> &chosen_screws)
        : home(std::move(chosen_home))
        , screws(chosen_screws.begin(), chosen_screws.end())
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
        , m_home_euler_degrees(Eigen::Vector3f::Zero())
        , m_stencil(target)
        , m_edits_cb(std::move(edits))
        , m_save_cb(std::move(save))
{
    seed(state);
}

// There is a row per joint of the derived chain, because that chain is what the rendered arm is
// posed from and what carries the limits a solve reads. A row nobody supplied opens degenerate, and
// an entry standing past the last joint is drawn against nothing and kept only to be counted.
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
    m_supplied = opened.screws;
    m_supplied.resize(std::max(m_derived.joint_count(), m_supplied.size()));
    for(std::size_t joint = 0u; joint < m_derived.joint_count(); ++joint)
    {
        const screw_axis drawn = supplied_or_opening(m_screw, m_derived.space_screws[joint], m_supplied[joint]);

        m_entries.push_back("Joint " + std::to_string(joint + 1u));
        m_rows.emplace_back(typed_as(drawn));
        m_rows.back().show(drawn);
    }
}

// The chain a caller takes back out is one entry per joint of the derived chain: an entry past the
// last joint stands against no joint and belongs in no document that names one.
screw_modeling_window::settings screw_modeling_window::state() const
{
    std::vector<supplied_screw> kept(m_supplied);
    kept.resize(m_derived.joint_count());

    return settings{m_home, kept};
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

    const std::vector<screw_axis> drawn = as_drawn(m_derived, m_screw, m_supplied);
    if(!m_stencil.set_joint_screws(m_home, drawn))
        spdlog::error("praxis: '{}' holds {} screws and the arm they are drawn against does not take that many", display_name(), drawn.size());
}

void screw_modeling_window::reset()
{
    m_home = transform::Identity();
    m_rows.clear();
    seed(settings{});
    push();
    tell_selection();
}

void screw_modeling_window::rebuild_row(std::size_t joint)
{
    const row &shown = m_rows[joint];
    if(shown.typed == parameterization::angular_linear)
    {
        m_supplied[joint] = m_screw.screw_axis_from_angular_linear(shown.angular.cast<double>(), shown.linear.cast<double>());

        return push();
    }

    if(shown.direction.isZero())
        return refuse(joint, "a direction of no length");
    const expected<screw_axis, refusal> built =
            m_screw.screw_axis_from_point_direction_pitch(shown.point.cast<double>(), shown.direction.cast<double>(), static_cast<double>(shown.pitch));
    if(!built)
        return refuse(joint, "'rigid_motion.screw.screw_axis_from_point_direction_pitch'");

    m_supplied[joint] = built.value();
    push();
}

void screw_modeling_window::refuse(std::size_t joint, const char *what)
{
    spdlog::error("praxis: {} named no axis for joint {} of '{}', so the screw it carried is kept", what, joint + 1u, display_name());
    canonicalize(joint);
}

void screw_modeling_window::canonicalize(std::size_t joint)
{
    m_rows[joint].show(supplied_or_opening(m_screw, m_derived.space_screws[joint], m_supplied[joint]));
}

}
