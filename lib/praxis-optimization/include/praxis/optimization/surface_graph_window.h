#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_GRAPH_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_GRAPH_WINDOW_H

#include "praxis/optimization/surface_stencil.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <string>

namespace praxis::optimization {

// Switches for what a surface's stencil draws in the scene: the field, the iterates, the zero curve,
// the selected step's construction, the box and the planes at the linearized point. A composition that opens no such window draws them
// all. The switches are a view of the scene and are not saved.
class surface_graph_window : public scene::imgui_window
{
public:
    surface_graph_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

private:
    std::shared_ptr<surface_view> m_drawn;
};

}

#endif
