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

transform read_home(const config::document &values, const std::string &at, const rigid_motion::frame_ops &framing)
{
    const Eigen::Vector3d position = keys::read_triple(values, keys::under(at, names::position), Eigen::Vector3d::Zero());
    const Eigen::Vector3d degrees  = keys::read_triple(values, keys::under(at, names::orientation), Eigen::Vector3d::Zero());
    const rotation held            = framing.rotation_matrix_from_euler(degrees * radians_per_degree, manipulator::screw_modeling_window::home_axis_order);

    return framing.transformation_matrix_from_rotation_position(held, position);
}

// A row the document carries an instance of is a row somebody wrote, and a leaf it leaves out of
// one is the zero the declaration falls back to -- which is exactly what a writer emitting only
// what moved leaves out. A row with no instance at all is a joint nobody supplied.
manipulator::supplied_screw read_row(const config::document &values, const std::string &collection, std::size_t joint)
{
    const std::optional<std::string> instance = keys::instance_at(values, collection, std::to_string(joint + 1u));
    if(!instance)
        return manipulator::supplied_screw();

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

// What one joint costs the document. An entry holding nothing takes out whatever row the document
// carries for it; one holding a screw is written because somebody supplied that joint rather than
// because a leaf of it moved, so a row the document has no instance of is named ahead of its own
// values and `appending` moves on to the ordinal the next such row lands at.
std::vector<config::edit> joint_edits(const config::document &values, const std::string &rows, const manipulator::supplied_screw &entry, std::size_t joint, std::size_t &appending)
{
    const std::string identity                = std::to_string(joint + 1u);
    const std::optional<std::string> instance = keys::instance_at(values, rows, identity);
    if(!entry && !instance)
        return {};
    if(!entry)
        return {config::edit{rows, identity, config::edit_kind::taken_out}};

    const std::string where           = instance ? *instance : rows + "[" + std::to_string(appending) + "]";
    std::vector<config::edit> written = config::unsaved_edits(values, row_edits(where, *entry));
    if(instance)
        return written;

    written.insert(written.begin(), config::edit{keys::under(where, names::index), identity});
    ++appending;

    return written;
}

// A removal for every row the document carries past the last of `entries` a state holds. A row
// whose identity names no joint's place is never one of them.
std::vector<config::edit> rows_past(const config::document &values, const std::string &rows, std::size_t entries)
{
    std::vector<config::edit> changes;
    for(const std::string &identity : values.identities(rows))
        if(const std::optional<std::size_t> named = keys::ordinal_of(identity); named && *named > entries)
            changes.push_back(config::edit{rows, identity, config::edit_kind::taken_out});

    return changes;
}

// A refusal naming the last joint `state` holds a screw for, where that joint lies further out than
// a document may name past the end of a chain of `joints`.
std::optional<config::error> beyond_reach(const supplied &state, std::size_t joints)
{
    const std::size_t furthest = joints + screw_table_greatest_surplus;
    for(std::size_t joint = state.screws.size(); joint > furthest; --joint)
        if(state.screws[joint - 1u])
            return config::error{config::error_code::rejected_content,
                                 "the chain handed here holds a screw for joint " + std::to_string(joint) + ", further past the end of this chain than joint " +
                                         std::to_string(furthest) + ", the furthest a document may name"};

    return std::nullopt;
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

expected<supplied, config::error> read_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived, const rigid_motion::screw_ops &,
                                                   const rigid_motion::frame_ops &framing)
{
    const std::string rows                           = keys::under(at, names::joint);
    const expected<std::size_t, config::error> reach = keys::reach_of(values.identities(rows), derived.joint_count());
    if(!reach)
        return unexpected(reach.error());

    supplied opened;
    opened.home = read_home(values, keys::under(at, names::home), framing);
    for(std::size_t joint = 0u; joint < *reach; ++joint)
        opened.screws.push_back(read_row(values, rows, joint));

    return opened;
}

expected<std::vector<config::edit>, config::error> write_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived, const supplied &state,
                                                                     const rigid_motion::frame_ops &framing)
{
    const std::string rows = keys::under(at, names::joint);
    if(const expected<std::size_t, config::error> reach = keys::reach_of(values.identities(rows), derived.joint_count()); !reach)
        return unexpected(reach.error());
    if(const std::optional<config::error> refused = beyond_reach(state, derived.joint_count()); refused)
        return unexpected(*refused);

    std::vector<config::edit> changes = config::unsaved_edits(values, home_edits(keys::under(at, names::home), state.home, framing));
    std::size_t appending             = values.identities(rows).size();
    for(std::size_t joint = 0u; joint < state.screws.size(); ++joint)
    {
        const std::vector<config::edit> row = joint_edits(values, rows, state.screws[joint], joint, appending);
        changes.insert(changes.end(), row.begin(), row.end());
    }

    const std::vector<config::edit> past = rows_past(values, rows, state.screws.size());
    changes.insert(changes.end(), past.begin(), past.end());

    return changes;
}

manipulator::screw_modeling_window::edit_route screw_table_edits(const manipulator::screw_chain &derived, const rigid_motion::frame_ops &framing)
{
    return [derived, framing](const config::document &values, std::string_view at, const supplied &state)
    {
        const expected<std::vector<config::edit>, config::error> changes = write_screw_table(values, at, derived, state, framing);
        if(changes)
            return changes.value();

        spdlog::error("praxis: the chain offers nothing on leaving: {}", changes.error().message);
        return std::vector<config::edit>();
    };
}

manipulator::screw_modeling_window::save_route screw_table_route(const std::optional<config::binding> &bound, const manipulator::screw_chain &derived,
                                                                 const rigid_motion::frame_ops &framing)
{
    if(!bound || bound->at.resolved.empty())
        return manipulator::screw_modeling_window::save_route();

    return [kept = *bound, derived, framing](std::string_view at, const supplied &state)
    {
        const config::outcome carried                                    = config::load_or_defaults(kept);
        const expected<std::vector<config::edit>, config::error> changes = write_screw_table(carried.values, at, derived, state, framing);
        const expected<void, config::error> written                      = changes ? config::save(kept, changes.value()) : expected<void, config::error>(unexpected(changes.error()));
        if(!written)
            spdlog::error("praxis: the chain was not kept: {}", written.error().message);
    };
}

}
