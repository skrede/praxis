#include "configuration_keys.h"

#include "praxis/manipulator/pose_configuration.h"

#include "praxis/config/configurable.h"

#include <array>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr std::array<const char *, 3> components{"x", "y", "z"};

struct pose_names
{
    static constexpr std::string_view euler_order = "euler_order";
    static constexpr std::string_view position    = "position";
    static constexpr std::string_view euler       = "euler";
};

const edited_pose &pose_fallbacks()
{
    static const edited_pose pose{};
    return pose;
}

bool carries_pose(const config::document &values, std::string_view at)
{
    for(const std::string_view vector : {pose_names::position, pose_names::euler})
        for(const char *component : components)
            if(keys::carried(values, keys::under(keys::under(at, vector), component)))
                return true;

    return false;
}

}

void declare_shared_pose(config::declaration &shape, std::string_view at)
{
    const edited_pose &was = pose_fallbacks();

    shape.group(std::string(at));
    keys::declare_order(shape, keys::under(at, pose_names::euler_order), was.order);
    keys::declare_vector(shape, keys::under(at, pose_names::position), was.position);
    keys::declare_vector(shape, keys::under(at, pose_names::euler), was.euler_degrees);
}

edited_pose read_shared_pose(const config::document &values, std::string_view at)
{
    const edited_pose &was = pose_fallbacks();

    edited_pose pose;
    pose.order         = keys::read_order(values, keys::under(at, pose_names::euler_order), was.order);
    pose.standing      = carries_pose(values, at) ? pose_standing::held : pose_standing::unset;
    pose.position      = keys::read_vector(values, keys::under(at, pose_names::position), was.position);
    pose.euler_degrees = keys::read_vector(values, keys::under(at, pose_names::euler), was.euler_degrees);
    return pose;
}

std::vector<config::edit> write_shared_pose(const edited_pose &pose, std::string_view at)
{
    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, pose_names::euler_order), keys::order_text(pose.order)});
    keys::write_vector(changes, keys::under(at, pose_names::position), pose.position);
    keys::write_vector(changes, keys::under(at, pose_names::euler), pose.euler_degrees);
    return changes;
}

std::vector<config::edit> unsaved_shared_pose(const config::document &carried, const edited_pose &pose, std::string_view at)
{
    if(pose.standing != pose_standing::held || at.empty())
        return {};

    if(!carries_pose(carried, at))
        return write_shared_pose(pose, at);

    return config::unsaved_edits(carried, keys::held_as_carried(carried, write_shared_pose(pose, at)));
}

}
