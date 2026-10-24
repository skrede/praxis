#include "praxis/optimization/display_window.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <utility>

namespace praxis::optimization {

display_window::display_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_surface(inputs.dimension == 2u)
        , m_minimum(inputs.opened.objective.form == problem_form::minimum)
        , m_drawn(inputs.drawn != nullptr ? inputs.drawn : std::make_shared<scenario_view>())
{
}

void display_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_surface)
        ImGui::Checkbox("Surface", &m_drawn->field);
    ImGui::Checkbox("Iterates", &m_drawn->iterates);
    if(m_surface && m_minimum)
    {
        ImGui::Checkbox("Level curves", &m_drawn->level_curves);
        ImGui::Checkbox("Gradient", &m_drawn->gradient);
    }
    else if(m_surface)
        ImGui::Checkbox("Zero curve", &m_drawn->zero_curve);
    ImGui::Checkbox("Step", &m_drawn->step);
    ImGui::Checkbox("Box", &m_drawn->bounds);
    if(m_surface && m_minimum)
        ImGui::Checkbox("Floor projection", &m_drawn->floor_projection);
    ImGui::End();
}

}
