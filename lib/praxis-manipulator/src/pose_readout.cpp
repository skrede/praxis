#include "praxis/manipulator/pose_readout.h"

#include "praxis/manipulator/option_widgets.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::manipulator {

namespace {

using pose_reading = pose_readout::pose_reading;

const char *words_for(pose_reading reading)
{
    switch(reading)
    {
        case pose_reading::value:
            return "";
        case pose_reading::unpublished:
            return "The arm has published nothing yet.";
        case pose_reading::refused:
            return "The pose was refused.";
        case pose_reading::position_unbound:
            return "The position is not bound.";
        case pose_reading::orientation_unbound:
            return "The orientation is not bound.";
        case pose_reading::both_unbound:
            return "The position and the orientation are not bound.";
    }

    return "";
}

std::vector<std::vector<scene::labeled_value>> rows_of(const Eigen::Vector3d &position, const Eigen::Vector3d &angles)
{
    const char *const position_labels[3]{"X", "Y", "Z"};
    const char *const angle_labels[3]{"A", "B", "C"};

    std::vector<std::vector<scene::labeled_value>> rows;
    rows.reserve(3);
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        rows.push_back({scene::labeled_value{static_cast<float>(position[axis]), position_labels[axis]},
                        scene::labeled_value{static_cast<float>(to_degrees(angles[axis])), angle_labels[axis]}});

    return rows;
}

}

pose_readout::pose_readout(arm_reader seen, const rigid_motion::frame_ops &injected, robot_slot_set inert)
        : m_seen(std::move(seen))
        , m_inert(inert)
        , m_euler_order(axis_order::zyx)
        , m_frame(injected)
        , m_frame_view(frame_view::tool, {frame_view::tool, frame_view::flange}, {"Tool", "Flange"})
{
}

void pose_readout::render_controls()
{
    render_option_cycle("Frame", m_frame_view);
    scene::render_enum_selection("Euler order", m_euler_order, axis_order_labels());
}

scene::readout pose_readout::reading() const
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();
    const frame_view frame                              = m_frame_view.value();
    const pose_reading answer                           = published ? reading_of(*published, frame) : pose_reading::unpublished;
    if(answer != pose_reading::value)
        return scene::readout{words_for(answer), {}};

    const bool flange               = frame == frame_view::flange;
    const Eigen::Vector3d &position = *(flange ? published->flange_position : published->tool_position);
    const rotation &orientation     = *(flange ? published->flange_orientation : published->tool_orientation);

    return scene::readout{std::string(), rows_of(position, m_frame.euler_from_rotation_matrix(orientation, m_euler_order))};
}

// The unbound answer is decided before the refusal one: an accessor the composition never bound is
// not reached by the poses below it, so a refusal it could have carried was never produced.
pose_readout::pose_reading pose_readout::reading_of(const arm_snapshot &seen, frame_view frame) const
{
    const bool position    = m_inert.contains(robot_slot::position_from_pose);
    const bool orientation = m_inert.contains(robot_slot::orientation_from_pose);
    if(position && orientation)
        return pose_reading::both_unbound;
    if(position)
        return pose_reading::position_unbound;
    if(orientation)
        return pose_reading::orientation_unbound;

    const bool flange = frame == frame_view::flange;
    if(!(flange ? seen.flange_position : seen.tool_position) || !(flange ? seen.flange_orientation : seen.tool_orientation))
        return pose_reading::refused;

    return pose_reading::value;
}

std::shared_ptr<scene::labeled_value_window> compose_pose_readout(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, robot_slot_set inert)
{
    const auto held = std::make_shared<pose_readout>(std::move(seen), injected, inert);

    return std::make_shared<scene::labeled_value_window>(std::move(name), [held] { held->render_controls(); }, [held] { return held->reading(); });
}

}
