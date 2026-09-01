#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_MODELING_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

// A chain supplied rather than derived: a home pose, one screw axis per joint, and a difference
// against the chain the description already carries. The rendered arm is posed from that
// description and moves correctly whichever screws are typed here, which is what leaves it standing
// as the reference a wrong screw is seen against.
class screw_modeling_window : public scene::imgui_window, public config::configurable
{
public:
    // The two published constructions a screw axis is built through, named for what a person types
    // rather than for a kind of joint.
    enum class parameterization : std::uint8_t
    {
        point_direction_pitch,
        angular_linear
    };

    // The chain a composition opens the window at: the home pose and one screw axis per joint, in
    // the frame of the model's root link. Which construction a row was typed in is not carried -- a
    // screw holds no such fact, and each row's is inferred from its own angular part.
    struct settings
    {
        transform home;
        std::vector<screw_axis> screws;

        explicit settings(transform chosen_home = transform::Identity(), std::vector<screw_axis> chosen_screws = std::vector<screw_axis>());
    };

    // Which controls a composition offers. A control this does not ask for is not drawn, so the
    // value behind it stays where the composition put it and nothing in the running application can
    // move it.
    struct controls
    {
        controls();

        bool home;
        bool reset;
    };

    // How the chain this window holds is spelled in a document, and where that spelling is handed.
    // Both are the composition's: nothing in this library holds a configuration binding, and the
    // wire format a supplied chain is kept in is named nowhere in it. A window handed no writer
    // offers nothing on leaving; one handed no route draws no save control.
    using edit_route = std::function<std::vector<config::edit>(const config::document &, std::string_view, const settings &)>;
    using save_route = std::function<void(std::string_view, const settings &)>;

    // The order the home pose's Euler triple is taken in, in degrees, which a reader of this
    // window's edits needs in order to read them back.
    static constexpr axis_order home_axis_order = axis_order::zyx;

    // The screw a row nobody supplied opens at: the unit z-axis through the origin, taken in
    // whichever of the two constructions the joint `derived` describes is typed in. Published so a
    // reader filling a row a document says nothing about fills it where this window would.
    static screw_axis opening_screw(const rigid_motion::screw_ops &turning, const screw_axis &derived);

    screw_modeling_window(std::string name, loadable_robot_stencil &target, arm_reader seen, const rigid_motion::screw_ops &turning, const rigid_motion::frame_ops &framing,
                          const forward_kinematics_ops &solving, screw_chain derived);
    screw_modeling_window(std::string name, loadable_robot_stencil &target, arm_reader seen, const rigid_motion::screw_ops &turning, const rigid_motion::frame_ops &framing,
                          const forward_kinematics_ops &solving, screw_chain derived, const controls &offered, const settings &state, edit_route edits, save_route save,
                          std::string at = std::string());

    settings state() const;

    // How far the supplied chain stands from the derived one at the published configuration, as a
    // rotation in radians and a distance in metres carried apart and never summed. A chain either
    // side declines to pose reads as a message in their place.
    scene::readout reading() const;

    // Whether the point a joint's row shows is the point the screw that row holds carries. A screw
    // stores the point of its axis nearest the origin, so a point typed anywhere else on the same
    // axis names the same line and is not the triple a reload returns. A row typed as an angular
    // and a linear part carries no point, and a joint this window holds no row for shows nothing.
    bool shows_stored_point(std::size_t joint) const;

    void render() override;

    void initialize() override;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    // A window no key path was named for, or none whose chain anything knows how to spell, has
    // nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() || !m_edits_cb ? nullptr : this;
    }

private:
    // One joint's numbers as each of the two constructions asks for them, with the one it is typed
    // in. The screw is not here: it is what the window stores and what the drawing is told.
    struct row
    {
        explicit row(parameterization opening);

        // Modern Robotics Eq. 3.24/3.25 read the other way: a screw with an angular part passes
        // through the point of its axis nearest the origin, which is the unit angular direction
        // crossed into the scaled linear part, and its pitch is that part's component along it.
        void show(const screw_axis &screw);

        float pitch;
        Eigen::Vector3f point;
        Eigen::Vector3f linear;
        Eigen::Vector3f angular;
        Eigen::Vector3f direction;
        option_cycle<parameterization, 2> typed;
    };

    transform m_home;
    arm_reader m_seen;
    controls m_controls;
    screw_chain m_derived;
    std::vector<row> m_rows;
    std::size_t m_selected;
    std::vector<std::string> m_entries;
    std::string m_settings_at;
    forward_kinematics_ops m_kinematics;
    Eigen::Vector3f m_home_position;
    rigid_motion::frame_ops m_frame;
    rigid_motion::screw_ops m_screw;
    // The construction each row is built through holds no refusal channel, so a composition that
    // left it at its default is said once for as long as that stands rather than once per push.
    bool m_unbound;
    std::vector<screw_axis> m_screws;
    Eigen::Vector3f m_home_euler_degrees;
    loadable_robot_stencil &m_stencil;
    edit_route m_edits_cb;
    save_route m_save_cb;

    void push();

    void reset();

    void render_home();

    void render_selector();

    void tell_selection();

    void render_save();

    void render_reading();

    void assemble_home();

    void seed(const settings &opened);

    void rebuild_row(std::size_t joint);

    void render_row(std::size_t joint);

    void refuse(std::size_t joint);

    void canonicalize(std::size_t joint);

    bool render_canonicalize(std::size_t joint);

    Eigen::Vector3f stored_point(std::size_t joint) const;
};

}

#endif
