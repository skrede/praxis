#include "praxis/optimization/surface_graph_window.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <utility>

namespace praxis::optimization {

// A curve's inputs carry no view, so a window composed over one switches a view nothing draws.
surface_graph_window::surface_graph_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_drawn(inputs.drawn != nullptr ? inputs.drawn : std::make_shared<surface_view>())
{
}

void surface_graph_window::render()
{
    ImGui::Begin(display_name().c_str());
    ImGui::Checkbox("Surface", &m_drawn->field);
    ImGui::Checkbox("Iterates", &m_drawn->iterates);
    ImGui::End();
}

}
