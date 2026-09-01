#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_WINDOW_H

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

class screw_window : public scene::imgui_window
{
public:
    // The pitch is the axis translation per radian, so the travel over a full turn is two pi times
    // the pitch.
    struct settings
    {
        Eigen::Vector3f point;
        Eigen::Vector3f direction;
        float pitch;
        float angle_radians;
    };

    // The route every piece of geometry derived from the screw is rebuilt through, invoked with what
    // the controls now say and with the axis those values name, and with nothing where they name
    // none: a scenario drawing more than one such thing rebuilds all of it from this one call, so the
    // drawn things cannot disagree about which axis they are about. These controls own no geometry:
    // what the scenario is made of is the scenario's own.
    using axis_route = std::function<void(const settings &shown, const std::optional<screw_axis> &named)>;

    // The three objects these controls drive, in the order the scenario composes them.
    static constexpr std::size_t axis_object   = 0;
    static constexpr std::size_t body_object   = 1;
    static constexpr std::size_t thread_object = 2;

    // The angle the slider reaches either way, and therefore the span the scenario draws its threads
    // over, so that the drawn geometry always covers the travel the controls can command.
    static constexpr double angle_limit_radians = 2.0 * std::numbers::pi;

    // The pose the moving object carries when this is built is the pose the screw is applied to, so a
    // scenario places that object before it composes its controls.
    screw_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild);
    screw_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen);

    settings state() const;

    void render() override;

    void initialize() override;

private:
    float m_pitch;
    float m_angle_radians;
    bool m_reported;
    transform m_start;
    capabilities m_motions;
    Eigen::Vector3f m_point;
    Eigen::Vector3f m_direction;
    std::optional<screw_axis> m_named;
    frame_stencil &m_stencil;
    axis_route m_rebuild_cb;

    void apply();
    void rebuild();
    void place_axis();
    void refuse(const char *what);
};

}

#endif
