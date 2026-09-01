#include "configuration_keys.h"

#include "praxis/manipulator/tool_configuration.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr std::array<const char *, 3> tool_views{"kinematics_transform", "graphics_transform", "load_stl"};
constexpr std::array<const char *, 2> world_views{"load_stl", "transform"};

// Every key these mappings address is one of these names joined onto the caller's path, so a
// declared key and the edit that writes it cannot spell themselves differently.
struct tool_names
{
    static constexpr std::string_view active      = "active";
    static constexpr std::string_view model       = "model";
    static constexpr std::string_view view        = "view";
    static constexpr std::string_view graphics    = "graphics";
    static constexpr std::string_view kinematics  = "kinematics";
    static constexpr std::string_view euler_order = "euler_order";
    static constexpr std::string_view euler       = "euler";
    static constexpr std::string_view scale       = "scale";
    static constexpr std::string_view offset      = "offset";
};

struct world_object_names
{
    static constexpr std::string_view active = "active";
    static constexpr std::string_view model  = "model";
    static constexpr std::string_view view   = "view";
    static constexpr std::string_view scale  = "scale";
    static constexpr std::string_view offset = "offset";
    static constexpr std::string_view euler  = "euler_zyx";
};

const tool_window::settings &tool_fallbacks()
{
    static const tool_window::settings state{};
    return state;
}

const world_object_window::settings &world_object_fallbacks()
{
    static const world_object_window::settings state{};
    return state;
}

}

void declare_tool(config::declaration &shape, std::string_view at)
{
    const tool_window::settings &was = tool_fallbacks();
    const std::string graphics       = keys::under(at, tool_names::graphics);
    const std::string kinematics     = keys::under(at, tool_names::kinematics);

    shape.group(std::string(at));
    shape.field(keys::under(at, tool_names::active), config::field_kind::flag, was.active ? "true" : "false");
    shape.field(keys::under(at, tool_names::model), config::field_kind::text, was.model_path);
    shape.choice(keys::under(at, tool_names::view), keys::spelled(tool_views), tool_views[static_cast<std::size_t>(was.selected_view)]);

    shape.group(graphics);
    keys::declare_order(shape, keys::under(graphics, tool_names::euler_order), was.gfx_euler_order);
    keys::declare_vector(shape, keys::under(graphics, tool_names::euler), was.gfx_euler_degrees);
    keys::declare_vector(shape, keys::under(graphics, tool_names::scale), was.gfx_scale);
    keys::declare_vector(shape, keys::under(graphics, tool_names::offset), was.gfx_offset);

    shape.group(kinematics);
    keys::declare_order(shape, keys::under(kinematics, tool_names::euler_order), was.kinematics_euler_order);
    keys::declare_vector(shape, keys::under(kinematics, tool_names::euler), was.kinematics_euler_degrees);
    keys::declare_vector(shape, keys::under(kinematics, tool_names::offset), was.kinematics_offset);
}

tool_window::settings read_tool(const config::document &values, std::string_view at)
{
    const tool_window::settings &was = tool_fallbacks();
    const std::string graphics       = keys::under(at, tool_names::graphics);
    const std::string kinematics     = keys::under(at, tool_names::kinematics);

    tool_window::settings state;
    state.active            = keys::flag_at(values, keys::under(at, tool_names::active), was.active);
    state.model_path        = keys::text_at(values, keys::under(at, tool_names::model), was.model_path);
    state.selected_view     = static_cast<tool_window::tool_view>(keys::indexed(values, keys::under(at, tool_names::view), tool_views, static_cast<std::size_t>(was.selected_view)));
    state.gfx_euler_order   = keys::read_order(values, keys::under(graphics, tool_names::euler_order), was.gfx_euler_order);
    state.gfx_euler_degrees = keys::read_vector(values, keys::under(graphics, tool_names::euler), was.gfx_euler_degrees);
    state.gfx_scale         = keys::read_vector(values, keys::under(graphics, tool_names::scale), was.gfx_scale);
    state.gfx_offset        = keys::read_vector(values, keys::under(graphics, tool_names::offset), was.gfx_offset);
    state.kinematics_euler_order   = keys::read_order(values, keys::under(kinematics, tool_names::euler_order), was.kinematics_euler_order);
    state.kinematics_euler_degrees = keys::read_vector(values, keys::under(kinematics, tool_names::euler), was.kinematics_euler_degrees);
    state.kinematics_offset        = keys::read_vector(values, keys::under(kinematics, tool_names::offset), was.kinematics_offset);
    return state;
}

std::vector<config::edit> write_tool(const tool_window::settings &state, std::string_view at)
{
    const std::string graphics   = keys::under(at, tool_names::graphics);
    const std::string kinematics = keys::under(at, tool_names::kinematics);

    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, tool_names::active), state.active ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, tool_names::model), state.model_path});
    changes.push_back(config::edit{keys::under(at, tool_names::view), tool_views[static_cast<std::size_t>(state.selected_view)]});
    changes.push_back(config::edit{keys::under(graphics, tool_names::euler_order), keys::order_text(state.gfx_euler_order)});
    keys::write_vector(changes, keys::under(graphics, tool_names::euler), state.gfx_euler_degrees);
    keys::write_vector(changes, keys::under(graphics, tool_names::scale), state.gfx_scale);
    keys::write_vector(changes, keys::under(graphics, tool_names::offset), state.gfx_offset);
    changes.push_back(config::edit{keys::under(kinematics, tool_names::euler_order), keys::order_text(state.kinematics_euler_order)});
    keys::write_vector(changes, keys::under(kinematics, tool_names::euler), state.kinematics_euler_degrees);
    keys::write_vector(changes, keys::under(kinematics, tool_names::offset), state.kinematics_offset);
    return changes;
}

void declare_world_object(config::declaration &shape, std::string_view at)
{
    const world_object_window::settings &was = world_object_fallbacks();

    shape.group(std::string(at));
    shape.field(keys::under(at, world_object_names::active), config::field_kind::flag, was.active ? "true" : "false");
    shape.field(keys::under(at, world_object_names::model), config::field_kind::text, was.model_path);
    shape.choice(keys::under(at, world_object_names::view), keys::spelled(world_views), world_views[static_cast<std::size_t>(was.selected_view)]);
    keys::declare_vector(shape, keys::under(at, world_object_names::scale), was.gfx_scale);
    keys::declare_vector(shape, keys::under(at, world_object_names::offset), was.gfx_offset);
    keys::declare_vector(shape, keys::under(at, world_object_names::euler), was.gfx_euler_zyx_degrees);
}

world_object_window::settings read_world_object(const config::document &values, std::string_view at)
{
    const world_object_window::settings &was = world_object_fallbacks();

    world_object_window::settings state;
    state.active     = keys::flag_at(values, keys::under(at, world_object_names::active), was.active);
    state.model_path = keys::text_at(values, keys::under(at, world_object_names::model), was.model_path);
    state.selected_view =
            static_cast<world_object_window::world_view>(keys::indexed(values, keys::under(at, world_object_names::view), world_views, static_cast<std::size_t>(was.selected_view)));
    state.gfx_scale             = keys::read_vector(values, keys::under(at, world_object_names::scale), was.gfx_scale);
    state.gfx_offset            = keys::read_vector(values, keys::under(at, world_object_names::offset), was.gfx_offset);
    state.gfx_euler_zyx_degrees = keys::read_vector(values, keys::under(at, world_object_names::euler), was.gfx_euler_zyx_degrees);
    return state;
}

std::vector<config::edit> write_world_object(const world_object_window::settings &state, std::string_view at)
{
    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, world_object_names::active), state.active ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, world_object_names::model), state.model_path});
    changes.push_back(config::edit{keys::under(at, world_object_names::view), world_views[static_cast<std::size_t>(state.selected_view)]});
    keys::write_vector(changes, keys::under(at, world_object_names::scale), state.gfx_scale);
    keys::write_vector(changes, keys::under(at, world_object_names::offset), state.gfx_offset);
    keys::write_vector(changes, keys::under(at, world_object_names::euler), state.gfx_euler_zyx_degrees);
    return changes;
}

}
