#include "configuration_keys.h"

#include "praxis/rigid_motion/frame_tree.h"
#include "praxis/rigid_motion/configuration.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::rigid_motion {

namespace {

config::error unnamed(std::string_view what, const std::string &given)
{
    return config::error{config::error_code::rejected_content, "the " + std::string(what) + " named '" + given + "' is no object of this arrangement"};
}

std::optional<std::size_t> index_of(std::span<const std::string> objects, const std::string &named)
{
    for(std::size_t which = 0u; which < objects.size(); ++which)
        if(objects[which] == named)
            return which;

    return std::nullopt;
}

// The relation the placements describe, run through the tree's own refusal, so a chain that would
// close a cycle is turned away by the check the tree already performs rather than a second copy.
std::optional<config::error> cyclic(std::span<const frame_window::placement> placed, std::span<const std::string> objects)
{
    frame_tree relation(placed.size(), frame_ops{});
    for(std::size_t which = 0u; which < placed.size(); ++which)
        if(!relation.set_parent(which, placed[which].parent))
            return config::error{config::error_code::rejected_content,
                                 "'" + keys::name_of(which, objects) + "' is placed in '" + keys::name_of(placed[which].parent, objects) + "', which would close a cycle"};

    return std::nullopt;
}

expected<frame_window::placement, config::error> read_placement(const config::document &values, const std::string &instance, std::span<const std::string> objects,
                                                                frame_window::placement one)
{
    one.order         = keys::read_order(values, keys::under(instance, keys::arrangement_names::order), one.order);
    one.position      = keys::read_vector(values, keys::under(instance, keys::arrangement_names::position), one.position);
    one.euler_degrees = keys::read_vector(values, keys::under(instance, keys::arrangement_names::euler), one.euler_degrees);

    const std::optional<std::string> above = keys::carried_text(values, keys::under(instance, keys::arrangement_names::parent));
    if(!above)
        return one;

    if(above->empty())
    {
        one.parent = std::nullopt;
        return one;
    }

    one.parent = index_of(objects, *above);
    if(!one.parent)
        return unexpected(unnamed(keys::arrangement_names::parent, *above));
    return one;
}

expected<std::vector<frame_window::placement>, config::error> read_objects(const config::document &values, const std::string &collection, std::span<const std::string> objects,
                                                                           const frame_window::settings &opening)
{
    std::vector<frame_window::placement> placed(objects.size());
    for(std::size_t which = 0u; which < objects.size(); ++which)
    {
        if(which < opening.objects.size())
            placed[which] = opening.objects[which];

        const std::optional<std::string> instance = keys::instance_at(values, collection, objects[which]);
        if(!instance)
            continue;

        const expected<frame_window::placement, config::error> one = read_placement(values, *instance, objects, placed[which]);
        if(!one)
            return unexpected(one.error());
        placed[which] = one.value();
    }
    return placed;
}

// What a composition of `objects` would open at against `values` as it now stands, or `opened`
// itself where `values` is refused, since that refusal is what left the composition at `opened`.
frame_window::settings standing(const config::document &values, std::string_view at, std::span<const std::string> objects, const frame_window::settings &opened)
{
    const expected<frame_window::settings, config::error> read = read_arrangement(values, at, objects, opened);

    return read ? read.value() : opened;
}

void write_object(std::vector<config::edit> &into, const std::string &where, bool named, const std::string &object, std::span<const config::edit> moved)
{
    if(!named)
        into.push_back(config::edit{keys::under(where, keys::arrangement_names::name), object});

    into.insert(into.end(), moved.begin(), moved.end());
}

}

void declare_arrangement(config::declaration &shape, std::string_view at)
{
    const std::string root(at);
    const std::string instance = keys::under(root, keys::arrangement_names::object);

    shape.group(root);
    shape.collection(instance, std::string(keys::arrangement_names::name));
    shape.field(keys::under(instance, keys::arrangement_names::parent), config::field_kind::text, std::string());
    keys::declare_order(shape, keys::under(instance, keys::arrangement_names::order), axis_order::zyx);
    keys::declare_vector(shape, keys::under(instance, keys::arrangement_names::position), Eigen::Vector3f::Zero());
    keys::declare_vector(shape, keys::under(instance, keys::arrangement_names::euler), Eigen::Vector3f::Zero());
}

expected<frame_window::settings, config::error> read_arrangement(const config::document &values, std::string_view at, std::span<const std::string> objects,
                                                                 const frame_window::settings &opening)
{
    const expected<std::vector<frame_window::placement>, config::error> placed = read_objects(values, keys::under(at, keys::arrangement_names::object), objects, opening);
    if(!placed)
        return unexpected(placed.error());

    if(const std::optional<config::error> closed = cyclic(placed.value(), objects); closed)
        return unexpected(*closed);

    frame_window::settings state;
    state.objects = placed.value();
    return state;
}

std::vector<config::edit> write_arrangement(const config::document &values, const frame_window::settings &opened, const frame_window::settings &state, std::string_view at,
                                            std::span<const std::string> objects)
{
    const std::string collection      = keys::under(at, keys::arrangement_names::object);
    const frame_window::settings open = standing(values, at, objects, opened);

    std::vector<config::edit> changes;
    std::size_t appended = values.identities(collection).size();
    for(std::size_t which = 0u; which < objects.size() && which < state.objects.size(); ++which)
    {
        const std::optional<std::string> instance = keys::instance_at(values, collection, objects[which]);
        const std::string where                   = instance ? *instance : collection + "[" + std::to_string(appended) + "]";
        const std::vector<config::edit> moved     = keys::moved_leaves(where, keys::placed_at(open, which), state.objects[which], objects);
        if(moved.empty())
            continue;

        write_object(changes, where, instance.has_value(), objects[which], moved);
        appended += instance ? 0u : 1u;
    }
    return changes;
}

}
