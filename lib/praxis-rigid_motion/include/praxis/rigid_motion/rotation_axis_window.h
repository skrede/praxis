#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_ROTATION_AXIS_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_ROTATION_AXIS_WINDOW_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"

#include <Eigen/Core>

#include <string>
#include <cstddef>
#include <numbers>
#include <optional>
#include <functional>

namespace praxis::rigid_motion {

class rotation_axis_window : public scene::imgui_window
{
public:
    // The direction names the axis the turn is taken about and the angle is that turn in radians,
    // so the two together spell one exponential coordinate. The four flags say which of the
    // scenario's drawings stand.
    struct settings
    {
        Eigen::Vector3f direction;
        float angle_radians;
        bool axis_shown;
        bool coordinate_shown;
        bool frame_shown;
        bool arc_shown;
    };

    // The route every piece of geometry derived from these controls is rebuilt through, invoked with
    // what the controls now say and with the unit axis they name, and with nothing where they name
    // none. These controls own no geometry: what the scenario is made of is the scenario's own.
    using axis_route = std::function<void(const settings &shown, const std::optional<Eigen::Vector3d> &named)>;

    // The one object these controls drive.
    static constexpr std::size_t frame_object = 0;

    // The angle the slider reaches either way, and therefore the turn the scenario draws over.
    static constexpr double angle_limit_radians = 2.0 * std::numbers::pi;

    // The pose the moving object carries when this is built is the pose the exponential is applied
    // to, so a scenario places that object before it composes its controls.
    rotation_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild);
    rotation_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen);

    settings state() const;

    void render() override;

    void initialize() override;

private:
    bool m_reported;
    settings m_shown;
    transform m_start;
    capabilities m_motions;
    std::optional<settings> m_applied;

    // A unit vector, which is why the turn the frame takes is the angle control's own number rather
    // than that number scaled by the length of the direction typed into the controls.
    std::optional<Eigen::Vector3d> m_named;
    frame_stencil &m_stencil;
    axis_route m_rebuild_cb;

    void apply();
    void settle();
    void refuse(const char *what);
};

}

#endif
