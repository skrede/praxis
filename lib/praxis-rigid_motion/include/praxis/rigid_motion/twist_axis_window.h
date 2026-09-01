#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_TWIST_AXIS_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_TWIST_AXIS_WINDOW_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/screw_window.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include <Eigen/Core>

#include <string>
#include <cstddef>
#include <optional>
#include <functional>

namespace praxis::rigid_motion {

class twist_axis_window : public scene::imgui_window
{
public:
    // The angular part is in radians per unit of the angle below and the linear part in metres per
    // that same unit, so the two together name a screw axis whose pitch is metres per radian.
    struct settings
    {
        Eigen::Vector3f angular_part;
        Eigen::Vector3f linear_part;
        float angle_radians;
    };

    // The route the geometry standing for the axis is rebuilt through, invoked with the axis the
    // twist names and with nothing where it names none. These controls own no geometry: what the
    // scenario is made of is the scenario's own.
    using axis_route = std::function<void(const std::optional<screw_axis> &named)>;

    // The two objects these controls drive, in the order the scenario composes them.
    static constexpr std::size_t axis_object = 0;
    static constexpr std::size_t body_object = 1;

    // The pose the moving object carries when this is built is the pose the exponential is applied
    // to, so a scenario places that object before it composes its controls.
    twist_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild);
    twist_axis_window(std::string name, frame_stencil &target, const capabilities &injected, axis_route rebuild, const settings &chosen);

    settings state() const;

    // One unlabeled row of the six components of the axis the bound operations named, which the
    // generic window draws as an aligned row. A twist naming no axis has no components to show and
    // reads as a message in their place.
    scene::readout reading() const;

    void render() override;

    void initialize() override;

private:
    bool m_reported;
    settings m_shown;
    transform m_start;
    capabilities m_motions;
    std::optional<settings> m_applied;
    std::optional<screw_axis> m_named;
    frame_stencil &m_stencil;
    axis_route m_rebuild_cb;

    void apply();
    void settle();
    void rebuild();
    void refuse(const char *what);
};

}

#endif
