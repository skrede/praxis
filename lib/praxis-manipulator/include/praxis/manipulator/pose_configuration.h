#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_POSE_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_POSE_CONFIGURATION_H

#include "praxis/manipulator/edited_pose.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

// The pose is one group at `at` holding its Euler order, its position in metres and its angles in
// degrees; it reads held where the document carries any of the six numbers.
void declare_shared_pose(config::declaration &shape, std::string_view at);
edited_pose read_shared_pose(const config::document &values, std::string_view at);
std::vector<config::edit> write_shared_pose(const edited_pose &pose, std::string_view at);

// Nothing unless the pose is held and `at` names a path; the whole pose where `carried` holds none of
// it, and otherwise only what differs.
std::vector<config::edit> unsaved_shared_pose(const config::document &carried, const edited_pose &pose, std::string_view at);

}

#endif
