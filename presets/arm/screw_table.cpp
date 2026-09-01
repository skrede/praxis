#include "screw_table_keys.h"

#include "praxis/presets/screw_table.h"

#include "praxis/config/store.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/angles.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::presets {

namespace {

using names    = keys::screw_table_names;
using supplied = manipulator::screw_modeling_window::settings;

config::error mismatched(std::size_t rows, std::size_t joints)
{
    return config::error{config::error_code::rejected_content,
                         "the chain kept here names " + std::to_string(rows) + " joints and the machine it was opened against has " + std::to_string(joints)};
}

bool within(const std::string &identity, std::size_t joints)
{
    for(std::size_t joint = 0u; joint < joints; ++joint)
        if(identity == std::to_string(joint + 1u))
            return true;

    return false;
}

transform read_home(const config::document &values, const std::string &at, const rigid_motion::frame_ops &framing)
{
    const Eigen::Vector3d position = keys::read_triple(values, keys::under(at, names::position), Eigen::Vector3d::Zero());
    const Eigen::Vector3d degrees  = keys::read_triple(values, keys::under(at, names::orientation), Eigen::Vector3d::Zero());
    const rotation held            = framing.rotation_matrix_from_euler(degrees * radians_per_degree, manipulator::screw_modeling_window::home_axis_order);

    return framing.transformation_matrix_from_rotation_position(held, position);
}

// A row the document carries an instance of is a row somebody wrote, and a leaf it leaves out of
// one is the zero the declaration falls back to -- which is exactly what a writer emitting only
// what moved leaves out. Only a row with no instance at all opens at the degenerate screw.
screw_axis read_row(const config::document &values, const std::string &collection, std::size_t joint, const screw_axis &opening)
{
    const std::optional<std::string> instance = keys::instance_at(values, collection, std::to_string(joint + 1u));
    if(!instance)
        return opening;

    screw_axis read;
    read.head<3>() = keys::read_triple(values, keys::under(*instance, names::angular), Eigen::Vector3d::Zero());
    read.tail<3>() = keys::read_triple(values, keys::under(*instance, names::linear), Eigen::Vector3d::Zero());

    return read;
}

std::vector<config::edit> home_edits(const std::string &at, const transform &home, const rigid_motion::frame_ops &framing)
{
    const rotation held         = home.block<3, 3>(0, 0);
    const Eigen::Vector3d taken = framing.euler_from_rotation_matrix(held, manipulator::screw_modeling_window::home_axis_order) * degrees_per_radian;

    std::vector<config::edit> changes;
    keys::write_shortest(changes, keys::under(at, names::position), home.block<3, 1>(0, 3).cast<float>());
    keys::write_shortest(changes, keys::under(at, names::orientation), taken.cast<float>());

    return changes;
}

std::vector<config::edit> row_edits(const std::string &where, const screw_axis &screw)
{
    std::vector<config::edit> changes;
    keys::write_exact(changes, keys::under(where, names::angular), screw.head<3>());
    keys::write_exact(changes, keys::under(where, names::linear), screw.tail<3>());

    return changes;
}

}

config::declaration screw_table_keyspace()
{
    const std::string root(screw_table_path);
    const std::string home = keys::under(root, names::home);
    const std::string rows = keys::under(root, names::joint);

    config::declaration shape("screw_table");
    shape.group(root);
    shape.group(home);
    keys::declare_triple(shape, keys::under(home, names::position));
    keys::declare_triple(shape, keys::under(home, names::orientation));
    shape.collection(rows, std::string(names::index));
    keys::declare_triple(shape, keys::under(rows, names::angular));
    keys::declare_triple(shape, keys::under(rows, names::linear));

    return shape;
}

config::binding screw_table_binding(const std::filesystem::path &named, const std::filesystem::path &beside)
{
    return config::binding{screw_table_keyspace(), config::resolve(named, beside), config::expectation::partial};
}

expected<supplied, config::error> read_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived, const rigid_motion::screw_ops &turning,
                                                   const rigid_motion::frame_ops &framing)
{
    const std::string rows                 = keys::under(at, names::joint);
    const std::vector<std::string> present = values.identities(rows);
    for(const std::string &identity : present)
        if(!within(identity, derived.joint_count()))
            return unexpected(mismatched(present.size(), derived.joint_count()));

    supplied opened;
    opened.home = read_home(values, keys::under(at, names::home), framing);
    for(std::size_t joint = 0u; joint < derived.joint_count(); ++joint)
        opened.screws.push_back(read_row(values, rows, joint, manipulator::screw_modeling_window::opening_screw(turning, derived.space_screws[joint])));

    return opened;
}

std::vector<config::edit> write_screw_table(const config::document &values, std::string_view at, const supplied &state, const rigid_motion::frame_ops &framing)
{
    const std::string rows            = keys::under(at, names::joint);
    std::vector<config::edit> changes = config::unsaved_edits(values, home_edits(keys::under(at, names::home), state.home, framing));

    std::size_t appended = values.identities(rows).size();
    for(std::size_t joint = 0u; joint < state.screws.size(); ++joint)
    {
        const std::optional<std::string> instance = keys::instance_at(values, rows, std::to_string(joint + 1u));
        const std::string where                   = instance ? *instance : rows + "[" + std::to_string(appended) + "]";
        const std::vector<config::edit> moved     = config::unsaved_edits(values, row_edits(where, state.screws[joint]));
        if(moved.empty())
            continue;

        if(!instance)
            changes.push_back(config::edit{keys::under(where, names::index), std::to_string(joint + 1u)});
        appended += instance ? 0u : 1u;
        changes.insert(changes.end(), moved.begin(), moved.end());
    }

    return changes;
}

manipulator::screw_modeling_window::edit_route screw_table_edits(const rigid_motion::frame_ops &framing)
{
    return [framing](const config::document &values, std::string_view at, const supplied &state) { return write_screw_table(values, at, state, framing); };
}

manipulator::screw_modeling_window::save_route screw_table_route(const std::optional<config::binding> &bound, const rigid_motion::frame_ops &framing)
{
    if(!bound || bound->at.resolved.empty())
        return manipulator::screw_modeling_window::save_route();

    return [kept = *bound, framing](std::string_view at, const supplied &state)
    {
        const config::outcome carried               = config::load_or_defaults(kept);
        const expected<void, config::error> written = config::save(kept, write_screw_table(carried.values, at, state, framing));
        if(!written)
            spdlog::error("praxis: the chain was not kept: {}", written.error().message);
    };
}

}
