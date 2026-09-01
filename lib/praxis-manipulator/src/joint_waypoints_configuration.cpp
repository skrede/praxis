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

constexpr std::string_view joints_leaf = "joints";

joint_vector mapped(const std::vector<double> &values)
{
    return joint_vector(Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size())));
}

std::string spelled(const joint_vector &row)
{
    std::vector<std::string> numbers;
    for(Eigen::Index joint = 0; joint < row.size(); ++joint)
        numbers.push_back(config::exact_text(row[joint]));

    return waypoint_rows::joined(numbers);
}

}

void declare_joint_waypoints(config::declaration &shape, std::string_view at)
{
    waypoint_rows::declare(shape, at, joints_leaf);
}

joint_waypoint_list::settings read_joint_waypoints(const config::document &values, std::string_view at, std::size_t joints)
{
    std::vector<joint_vector> rows;
    for(const waypoint_rows::carried_row &carried : waypoint_rows::read(values, at, joints_leaf))
    {
        if(carried.values.size() == joints)
        {
            rows.push_back(mapped(carried.values));
            continue;
        }

        spdlog::error("praxis: 'manipulator.read_joint_waypoints' was given a row of {} joint values at row {} for an arm of {} joints, so it was not read into the list and the "
                      "rows beside it still are",
                      carried.values.size(), carried.identity, joints);
    }

    return joint_waypoint_list::settings{std::move(rows)};
}

std::vector<config::edit> list_row_traits<joint_vector>::written(const config::document &values, const std::vector<joint_vector> &rows, std::string_view at)
{
    std::vector<std::string> spelled_rows;
    for(const joint_vector &row : rows)
        spelled_rows.push_back(spelled(row));

    return waypoint_rows::write(values, spelled_rows, at, joints_leaf);
}

std::vector<config::edit> write_joint_waypoints(const config::document &values, const joint_waypoint_list::settings &state, std::string_view at)
{
    return list_row_traits<joint_vector>::written(values, state.rows, at);
}

}
