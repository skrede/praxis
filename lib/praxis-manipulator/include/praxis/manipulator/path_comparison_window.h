#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/trajectory/path.h"

#include <Eigen/Core>

#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

enum class compared_path : std::uint8_t
{
    joint_space,
    decoupled,
    screw
};

using compared_path_cycle = option_cycle<compared_path, 3>;

compared_path_cycle every_compared_path(compared_path chosen);

// Three paths over one pair of configurations, drawn at once. The ends are configurations, so the
// two poses the task-space shapes run between come from the forward map and no solve enters any of
// the three drawings. Every pose here is the forward map's own, which is the flange frame; a
// composition attaching no tool leaves that the frame the traversed path is recorded in too.
class path_comparison_window : public scene::imgui_window, public config::configurable
{
public:
    struct settings
    {
        joint_vector first;
        joint_vector second;
        bool joint_space;
        bool decoupled;
        bool screw;
        compared_path played;

        settings(joint_vector opening_first = joint_vector(), joint_vector opening_second = joint_vector(), bool drawing_joint_space = true, bool drawing_decoupled = true,
                 bool drawing_screw = true, compared_path chosen = compared_path::screw);
    };

    // The names the three drawings stand under on the stencil. They are orthogonal drawings: hiding
    // one reaches neither of the others.
    static constexpr const char *joint_space_path = "Joint space";
    static constexpr const char *decoupled_path   = "Decoupled";
    static constexpr const char *screw_path       = "Screw";

    // How many points every one of the three polylines is drawn from. All three carry this count and
    // are sampled at the same path parameters: two polylines sampled at different parameters are not
    // comparable point for point.
    static constexpr std::size_t drawn_points = 65;

    // The two ends a comparison over `joints` joints opens at where a document names none. An empty
    // pair in a settings value is the absence of a choice rather than a pair of no joints.
    static joint_vector opening_first(std::size_t joints);
    static joint_vector opening_second(std::size_t joints);

    path_comparison_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, forward_kinematics_ops forward, screw_chain chain,
                           trajectory::path_ops shapes);
    path_comparison_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, loadable_robot_stencil &drawn, forward_kinematics_ops forward, screw_chain chain,
                           trajectory::path_ops shapes, const settings &state, std::string at = std::string());

    settings state() const;

    // The three polylines the ends held here give, in the same path parameters and the same count.
    // Empty where the forward map or the shape refused, so a caller draws what it was given rather
    // than a run with holes in it.
    std::vector<transform> poses_along(compared_path shape) const;

    // The drawings are told here rather than from the controls, so a collapsed panel still draws the
    // three shapes its ends describe and the run the tool traversed over whichever of them played.
    void render() override;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    // A window no key path was named for has nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() ? nullptr : this;
    }

private:
    bool m_screw;
    bool m_decoupled;
    bool m_joint_space;
    arm_reader m_seen;
    screw_chain m_chain;
    std::string m_settings_at;
    forward_kinematics_ops m_fk;
    trajectory::path_ops m_shapes;
    compared_path_cycle m_played;
    std::weak_ptr<owned_arm> m_arm;
    loadable_robot_stencil &m_drawn;
    // Each end in degrees, which is the unit the ends are read and typed in; `state()` is where the
    // radians every other surface carries are taken.
    Eigen::VectorXf m_first;
    Eigen::VectorXf m_second;
    // What the drawing was last told, held so that three polylines of sixty-five poses are rebuilt
    // where an end changed and not on every frame.
    Eigen::VectorXf m_told_first;
    Eigen::VectorXf m_told_second;
    bool m_told;
    // The traversed run the stencil was last told, held to compare the published handle against by
    // pointer identity: it is replaced whole and never appended to, so an unchanged handle is an
    // unchanged run and rebuilding a polyline from it would cost a thousand poses a frame.
    std::shared_ptr<const std::vector<transform>> m_told_traversed;

    void accept(const joint_vector &end, std::string_view named, std::size_t joints, Eigen::VectorXf &into);
    void render_controls();
    void render_ends();
    void render_switches();
    void draw_paths();
    void draw_traversed();
    void show_paths();
    void play();
};

}

#endif
