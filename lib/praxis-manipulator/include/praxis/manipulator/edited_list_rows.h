#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_ROWS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_ROWS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include "praxis/rigid_motion/frame.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

// Everything one kind of row does that another kind does not. A list of rows adds, removes,
// reorders, captures, declines and persists through this and differs nowhere else.
template<typename Row>
struct list_row_traits;

template<>
struct list_row_traits<joint_vector>
{
    // Degrees, which is the unit the row's own controls are read and typed in.
    using shown_row = Eigen::VectorXf;

    static shown_row shown(const joint_vector &row, const rigid_motion::frame_ops &frames);
    static joint_vector taken(const shown_row &row, const rigid_motion::frame_ops &frames);

    // What is wrong with the row, and nothing at all where it is a row a list on this arm can hold.
    static std::optional<std::string> fault_of(const joint_vector &row, std::size_t joints);

    // What stopped the publication from filling `into`, and nothing at all where it filled it.
    static std::optional<std::string> captured(shown_row &into, const arm_snapshot &seen, const rigid_motion::frame_ops &frames);

    // What each value of a row is called, one name per column: the arm's joints, named as they are
    // wherever one is drawn beside the others.
    static std::vector<std::string> column_labels(std::size_t joints);

    // True where the controls were typed into on the frame they were drawn.
    static bool render(shown_row &row);

    static std::vector<joint_vector> opening_rows(std::size_t joints);
    static std::vector<config::edit> written(const config::document &values, const std::vector<joint_vector> &rows, std::string_view at);
};

template<>
struct list_row_traits<edited_pose>
{
    // Metres and degrees, which the row already holds, so a pose is shown in the unit it is kept in.
    using shown_row = edited_pose;

    static shown_row shown(const edited_pose &row, const rigid_motion::frame_ops &frames);
    static edited_pose taken(const shown_row &row, const rigid_motion::frame_ops &frames);

    // Six numbers are six numbers whatever the arm is, so a pose row is never of the wrong width.
    static std::optional<std::string> fault_of(const edited_pose &row, std::size_t joints);

    static std::optional<std::string> captured(shown_row &into, const arm_snapshot &seen, const rigid_motion::frame_ops &frames);

    // The three position components and the three Euler angles, in the order the row draws them.
    static std::vector<std::string> column_labels(std::size_t joints);

    static bool render(shown_row &row);

    static std::vector<edited_pose> opening_rows(std::size_t joints);
    static std::vector<config::edit> written(const config::document &values, const std::vector<edited_pose> &rows, std::string_view at);
};

}

#endif
