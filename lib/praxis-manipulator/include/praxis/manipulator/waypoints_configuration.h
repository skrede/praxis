#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_WAYPOINTS_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_WAYPOINTS_CONFIGURATION_H

#include "praxis/manipulator/edited_list_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. Each of these declares its own leaves beneath it and declares
// nothing above it. A document carrying no row at all leaves the list opening at the rows the
// window's own `opening_rows` names.

// The rows are a collection keyed by their position in the list, counted from one, and each row
// carries its joint values in radians. A row whose width is not `joints` is declined by name and the
// rows beside it are still read. The write takes the document because a document carries no way to
// drop a row: a list shorter than the one already there empties the rows it no longer reaches, and a
// row carrying no value is read as one that is not there.
void declare_joint_waypoints(config::declaration &shape, std::string_view at);
joint_waypoint_list::settings read_joint_waypoints(const config::document &values, std::string_view at, std::size_t joints);
std::vector<config::edit> write_joint_waypoints(const config::document &values, const joint_waypoint_list::settings &state, std::string_view at);

// The same, each row carrying three metres of position and then three degrees of orientation, both
// in the space frame and the angles read in the axis order an edited pose opens at. Six numbers are
// six numbers whatever the arm is, so no joint count enters.
void declare_pose_waypoints(config::declaration &shape, std::string_view at);
pose_waypoint_list::settings read_pose_waypoints(const config::document &values, std::string_view at);
std::vector<config::edit> write_pose_waypoints(const config::document &values, const pose_waypoint_list::settings &state, std::string_view at);

}

#endif
