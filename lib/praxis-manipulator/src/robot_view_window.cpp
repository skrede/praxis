#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/view_configuration.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <span>
#include <array>
#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// A line of no length draws nothing while still reporting a reach, so the control admits no value
// below this. In metres.
constexpr float smallest_reach = 0.001f;

constexpr std::array<const char *, 4> model_labels{"Meshes", "Joint chain", "Meshes and chain", "None"};

}

std::span<const char *const> model_render_labels()
{
    return std::span<const char *const>(model_labels);
}

bool model_render_draws_meshes(model_render which)
{
    return which == model_render::meshes || which == model_render::meshes_and_chain;
}

bool model_render_draws_chain(model_render which)
{
    return which == model_render::chain || which == model_render::meshes_and_chain;
}

robot_view_window::settings::settings(model_render chosen_model, bool chosen_decoration, std::optional<double> chosen_reach)
        : model(chosen_model)
        , decoration(chosen_decoration)
        , axis_reach(chosen_reach)
{
}

robot_view_window::robot_view_window(std::string name, loadable_robot_stencil &target)
        : robot_view_window(std::move(name), target, controls(), settings{})
{
}

robot_view_window::robot_view_window(std::string name, loadable_robot_stencil &target, const controls &offered, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_reach(std::max(smallest_reach, static_cast<float>(state.axis_reach.value_or(target.decoration_reach()))))
        , m_decoration(state.decoration)
        , m_model(state.model)
        , m_reach_named(state.axis_reach.has_value())
        , m_controls(offered)
        , m_settings_at(std::move(at))
        , m_stencil(target)
{
}

robot_view_window::settings robot_view_window::state() const
{
    return settings{m_model, m_decoration, m_reach_named ? std::optional<double>(m_reach) : std::nullopt};
}

std::vector<config::edit> robot_view_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_robot_view(state(), m_settings_at));
}

void robot_view_window::show_model()
{
    m_stencil.set_meshes_shown(model_render_draws_meshes(m_model));
    m_stencil.set_chain_shown(model_render_draws_chain(m_model));
}

// Every one of the three reaches the stencil whether or not a control was drawn for it, which is
// what leaves a feature nobody offered a control for standing where the composition put it.
void robot_view_window::initialize()
{
    show_model();
    m_stencil.set_decoration_shown(m_decoration);
    m_stencil.set_decoration_reach(static_cast<double>(m_reach));
}

void robot_view_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_controls.model)
        render_model();
    if(m_controls.decoration)
        render_decoration();

    if(m_controls.reach && (m_controls.model || m_controls.decoration))
        ImGui::Separator();
    if(m_controls.reach)
        render_reach();
    ImGui::End();
}

// An arm the stencil holds no chain for cannot draw one, so the entries that would are not offered
// over it and the control names only the drawings that exist.
void robot_view_window::render_model()
{
    const bool chained  = m_stencil.holds_chain();
    const auto drawable = [chained](model_render which) { return chained || !model_render_draws_chain(which); };

    if(scene::render_enum_selection("Model render", m_model, model_render_labels(), drawable))
        show_model();
}

void robot_view_window::render_decoration()
{
    if(ImGui::Checkbox("Screw axes", &m_decoration))
        m_stencil.set_decoration_shown(m_decoration);
}

void robot_view_window::render_reach()
{
    if(!ImGui::InputFloat("Axis reach (m)", &m_reach, 0.05f, 0.25f))
        return;

    m_reach       = std::max(m_reach, smallest_reach);
    m_reach_named = true;
    m_stencil.set_decoration_reach(static_cast<double>(m_reach));
}

}
