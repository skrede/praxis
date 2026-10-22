#include "praxis/optimization/display_window.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <utility>

namespace praxis::optimization {

display_window::display_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_surface(inputs.dimension == 2u)
        , m_drawn(inputs.drawn != nullptr ? inputs.drawn : std::make_shared<scenario_view>())
{
}

void display_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_surface)
        ImGui::Checkbox("Surface", &m_drawn->field);
    ImGui::Checkbox("Iterates", &m_drawn->iterates);
    if(m_surface)
        ImGui::Checkbox("Zero curve", &m_drawn->zero_curve);
    ImGui::Checkbox("Step", &m_drawn->step);
    ImGui::Checkbox("Box", &m_drawn->bounds);
    ImGui::End();
}

}
