#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_TWO_POSE_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_TWO_POSE_WINDOW_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include <Eigen/Core>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <functional>

namespace praxis::rigid_motion {

class two_pose_window : public scene::imgui_window
{
public:
    // One pose in the frame the stencil's poses are written in: a position in metres and the three
    // Euler angles in degrees, taken in the order named beside them.
    struct pose_controls
    {
        Eigen::Vector3f position;
        Eigen::Vector3f euler_degrees;
        axis_order order;
    };

    // The parameter is the share of the screw's magnitude the moving object has travelled, in the
    // closed unit range: zero leaves it at the start pose and one carries it to the end pose.
    struct settings
    {
        pose_controls start;
        pose_controls end;
        float parameter;
    };

    // The route the geometry standing for the travelled path is rebuilt through, invoked with the
    // poses sampled along that path and with an empty span where there is no path to draw. These
    // controls own no geometry: what the scenario is made of is the scenario's own.
    using path_route = std::function<void(std::span<const transform> travelled)>;

    // The four objects these controls drive, in the order the scenario composes them. The path
    // object is the scenario's to fill through the route; the other three are placed here.
    static constexpr std::size_t start_object = 0;
    static constexpr std::size_t end_object   = 1;
    static constexpr std::size_t body_object  = 2;
    static constexpr std::size_t path_object  = 3;

    // The poses sampled along the path, both ends included.
    static constexpr std::size_t path_points = 33;

    two_pose_window(std::string name, frame_stencil &target, const capabilities &injected, path_route rebuild);
    two_pose_window(std::string name, frame_stencil &target, const capabilities &injected, path_route rebuild, const settings &chosen);

    settings state() const;

    // The two poses the controls now name, in the frame the stencil's poses are written in, which is
    // what a second window drawing between the same two poses reads rather than naming them again.
    transform start_pose() const;
    transform end_pose() const;

    // One row carrying the length of the path last drawn. Two poses naming no motion have no path to
    // measure and read as a message in its place.
    scene::readout reading() const;

    void render() override;

    void initialize() override;

private:
    bool m_refused;
    bool m_reported;
    settings m_shown;
    double m_length;
    double m_magnitude;
    capabilities m_motions;
    std::string m_message;
    std::optional<screw_axis> m_named;
    std::optional<transform> m_applied_start;
    std::optional<transform> m_applied_end;
    frame_stencil &m_stencil;
    path_route m_rebuild_cb;

    transform posed(const pose_controls &shown) const;
    std::vector<transform> sampled(const transform &start) const;

    void place();
    void settle();
    void render_reading();
    void render_pose(pose_controls &shown, int id);
    void name(const transform &start, const transform &end);
    void rebuild(const transform &start, const transform &end);
    void refuse(const char *what);
};

}

#endif
