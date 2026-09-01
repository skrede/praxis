#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_roster_window.h"

#include <imgui.h>

#include <string>
#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::rigid_motion {

namespace {

const char *refusal_name(refusal reason)
{
    switch(reason)
    {
        case refusal::unsupported_input:
            return "unsupported input";
        case refusal::degenerate:
            return "degenerate";
        case refusal::no_solution:
            return "no solution";
        case refusal::not_implemented:
            return "not implemented";
    }

    return "unclassified";
}

std::string refused(const std::string &named, const char *what, refusal reason)
{
    return "'" + named + "' " + what + " (" + refusal_name(reason) + ")";
}

}

frame_roster_window::frame_roster_window(std::string name, frame_stencil &target, axes_settings arriving, object_body carried, std::string stem, selection_route standing)
        : imgui_window(std::move(name))
        , m_generated(0)
        , m_arriving(arriving)
        , m_carried(std::move(carried))
        , m_stem(std::move(stem))
        , m_stencil(target)
        , m_standing(std::move(standing))
        , m_refused_create()
        , m_refused_remove()
        , m_typed()
{
}

expected<std::size_t, refusal> frame_roster_window::create(std::string_view named)
{
    m_refused_create.clear();

    const std::string chosen = named.empty() ? generated_name() : std::string(named);
    if(carried_already(chosen))
    {
        m_refused_create = refused(chosen, "is a name a frame already carries, so nothing was created", refusal::unsupported_input);

        return unexpected(refusal::unsupported_input);
    }

    const std::size_t made = m_stencil.add(stencil_object{chosen, m_arriving, m_carried});
    m_standing.write(made);

    return made;
}

expected<void, refusal> frame_roster_window::remove_selected()
{
    m_refused_remove.clear();

    const std::size_t standing = m_standing.read();
    const std::string named(m_stencil.name_of(standing));
    const expected<void, refusal> accepted = m_stencil.remove(standing);
    if(!accepted)
    {
        m_refused_remove = refused(named, "was left where it was", accepted.error());

        return accepted;
    }

    m_standing.write(standing);

    return {};
}

void frame_roster_window::render()
{
    ImGui::Begin(display_name().c_str());
    render_roster();
    render_creation();
    render_removal();
    ImGui::End();
}

void frame_roster_window::render_roster()
{
    const std::size_t standing = m_standing.read();
    for(std::size_t index = 0; index < m_stencil.count(); ++index)
    {
        const std::string named(m_stencil.name_of(index));

        ImGui::PushID(static_cast<int>(index));
        if(ImGui::Selectable(named.c_str(), index == standing))
            m_standing.write(index);
        ImGui::PopID();
    }
}

void frame_roster_window::render_creation()
{
    ImGui::InputText("Name", m_typed.data(), m_typed.size());
    if(ImGui::Button("Create") && create(std::string_view(m_typed.data())))
        m_typed.front() = '\0';
    if(!m_refused_create.empty())
        ImGui::TextUnformatted(m_refused_create.c_str());
}

void frame_roster_window::render_removal()
{
    if(ImGui::Button("Remove"))
        static_cast<void>(remove_selected());
    if(!m_refused_remove.empty())
        ImGui::TextUnformatted(m_refused_remove.c_str());
}

// The ordinal only moves forward, so a name is never handed out twice while this window lives and a
// name a removed frame carried is not given to a later one. The collision skip is what keeps a
// generated name clear of one that was typed.
std::string frame_roster_window::generated_name()
{
    std::string chosen;
    do
        chosen = m_stem + " " + std::to_string(++m_generated);
    while(carried_already(chosen));

    return chosen;
}

bool frame_roster_window::carried_already(std::string_view named) const
{
    if(m_stencil.fixed_frame_name() == named)
        return true;

    for(std::size_t index = 0; index < m_stencil.count(); ++index)
        if(m_stencil.name_of(index) == named)
            return true;

    return false;
}

}
