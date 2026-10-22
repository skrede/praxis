#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_DISPLAY_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_DISPLAY_WINDOW_H

#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/scene/imgui_window.h"

#include <memory>
#include <string>

namespace praxis::optimization {

// Switches for what the scenario's drawing shows: the iterates, the selected step's construction and
// the box, and over a surface the field and the zero curve as well. A composition that opens no such
// window draws them all. The switches are a view of the drawing and are not saved.
class display_window : public scene::imgui_window
{
public:
    display_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

private:
    bool m_surface;
    std::shared_ptr<scenario_view> m_drawn;
};

}

#endif
