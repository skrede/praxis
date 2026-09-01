#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_DECOUPLED_PATH_WINDOW_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_DECOUPLED_PATH_WINDOW_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/two_pose_window.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::rigid_motion {

// The path a body follows when its orientation and its position are carried between the two poses
// independently of each other, which is not the screw between them. A scenario that composes this
// has that second path and one that does not has neither it nor any control that could ask for it.
class decoupled_path_window : public scene::imgui_window
{
public:
    // The two poses this draws between, the start first and the end second, in the frame the
    // stencil's poses are written in. They are read from the scenario that composes both windows
    // rather than named again here, so the two paths cannot disagree about which poses they join.
    using pose_source = std::function<std::pair<transform, transform>()>;

    using path_route = two_pose_window::path_route;

    // The object these controls drive, composed after the four the coupled window names. It is the
    // scenario's to fill through the route.
    static constexpr std::size_t path_object = 4;

    decoupled_path_window(std::string name, const capabilities &injected, pose_source between, path_route rebuild);

    // One row carrying the length of the path last drawn, against which the coupled window's own
    // reading stands. Two poses naming no motion read as a message in its place.
    scene::readout reading() const;

    void render() override;

    void initialize() override;

private:
    bool m_reported;
    double m_length;
    double m_turn_radians;
    capabilities m_motions;
    std::string m_message;
    std::optional<Eigen::Vector3d> m_named;
    std::optional<transform> m_applied_start;
    std::optional<transform> m_applied_end;
    pose_source m_between;
    path_route m_rebuild_cb;

    std::vector<transform> sampled(const transform &start, const transform &end) const;

    void settle();
    void name(const transform &start, const transform &end);
    void rebuild(const transform &start, const transform &end);
    void refuse(const char *what);
};

}

#endif
