#include "configuration_keys.h"
#include "waypoint_rows_document.h"

#include "praxis/manipulator/waypoints_configuration.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr std::string_view pose_leaf = "pose";

// Three metres of position and then three degrees of orientation.
constexpr std::size_t pose_numbers = 6;

edited_pose posed(const std::vector<double> &values)
{
    edited_pose taken;
    taken.position      = Eigen::Vector3d(values[0], values[1], values[2]).cast<float>();
    taken.euler_degrees = Eigen::Vector3d(values[3], values[4], values[5]).cast<float>();

    return taken;
}

std::string spelled(const edited_pose &row)
{
    std::vector<std::string> numbers;
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        numbers.push_back(keys::text_of(row.position[axis]));
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        numbers.push_back(keys::text_of(row.euler_degrees[axis]));

    return waypoint_rows::joined(numbers);
}

}

void declare_pose_waypoints(config::declaration &shape, std::string_view at)
{
    waypoint_rows::declare(shape, at, pose_leaf);
}

pose_waypoint_list::settings read_pose_waypoints(const config::document &values, std::string_view at)
{
    std::vector<edited_pose> rows;
    for(const waypoint_rows::carried_row &carried : waypoint_rows::read(values, at, pose_leaf))
    {
        if(carried.values.size() == pose_numbers)
        {
            rows.push_back(posed(carried.values));
            continue;
        }

        spdlog::error("praxis: 'manipulator.read_pose_waypoints' was given a row of {} numbers at row {} where a pose is {}, so it was not read into the list and the rows beside "
                      "it still are",
                      carried.values.size(), carried.identity, pose_numbers);
    }

    return pose_waypoint_list::settings{std::move(rows)};
}

std::vector<config::edit> list_row_traits<edited_pose>::written(const config::document &values, const std::vector<edited_pose> &rows, std::string_view at)
{
    std::vector<std::string> spelled_rows;
    for(const edited_pose &row : rows)
        spelled_rows.push_back(spelled(row));

    return waypoint_rows::write(values, spelled_rows, at, pose_leaf);
}

std::vector<config::edit> write_pose_waypoints(const config::document &values, const pose_waypoint_list::settings &state, std::string_view at)
{
    return list_row_traits<edited_pose>::written(values, state.rows, at);
}

}
