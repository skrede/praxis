#include "praxis/manipulator/ik_convergence_window.h"
#include "praxis/manipulator/kinematics_configuration.h"

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::manipulator {

namespace {

constexpr const char *step_axis  = "Step";
constexpr const char *error_axis = "Error";

constexpr const char *nothing_chosen  = "Choose an error to draw.";
constexpr const char *nothing_stepped = "Ask for a solve to see how its error falls.";

// A logarithmic ordinate has no place for a value at or below zero, so a step that reached one is
// left out with its abscissa rather than raised to a value it did not have; the steps around it
// keep the positions they were taken at, which is what the abscissa carries them at.
scene::plot_series drawn(std::string name, const std::vector<iteration_state> &steps, double iteration_state::*error)
{
    scene::plot_series curve{std::move(name), {}, {}};
    for(const iteration_state &step : steps)
        if(step.*error > 0.0)
        {
            curve.abscissa.push_back(static_cast<double>(step.index));
            curve.ordinate.push_back(step.*error);
        }

    return curve;
}

bool empty_curve(const scene::plot_series &curve)
{
    return curve.ordinate.empty();
}

}

ik_convergence_window::settings::settings(bool chosen_angular, bool chosen_linear)
        : angular(chosen_angular)
        , linear(chosen_linear)
{
}

ik_convergence_window::ik_convergence_window(std::string name, const ik_iterate_window &followed)
        : ik_convergence_window(std::move(name), followed, settings{})
{
}

ik_convergence_window::ik_convergence_window(std::string name, const ik_iterate_window &followed, const settings &state, std::string at)
        : plot_window(
                  std::move(name), [this] { render_controls(); }, [this] { return reading(); })
        , m_linear(state.linear)
        , m_angular(state.angular)
        , m_settings_at(std::move(at))
        , m_followed(followed)
{
}

ik_convergence_window::settings ik_convergence_window::state() const
{
    return settings{m_angular, m_linear};
}

scene::plot_reading ik_convergence_window::reading() const
{
    const std::vector<iteration_state> steps = m_followed.iterates();

    scene::plot_frame errors{.ordinate_label = error_axis, .logarithmic_ordinate = true, .series = std::vector<scene::plot_series>()};
    if(m_angular)
        errors.series.push_back(drawn("Angular error", steps, &iteration_state::angular_error));
    if(m_linear)
        errors.series.push_back(drawn("Linear error", steps, &iteration_state::linear_error));

    scene::plot_reading shown{.message = std::string(), .abscissa_label = step_axis, .frames = std::vector<scene::plot_frame>()};
    if(std::all_of(errors.series.begin(), errors.series.end(), &empty_curve))
        shown.message = errors.series.empty() ? nothing_chosen : nothing_stepped;
    else
        shown.frames.push_back(std::move(errors));

    return shown;
}

std::vector<config::edit> ik_convergence_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_ik_convergence(state(), m_settings_at));
}

void ik_convergence_window::render_controls()
{
    ImGui::Checkbox("Angular error", &m_angular);
    ImGui::SameLine();
    ImGui::Checkbox("Linear error", &m_linear);
}

}
