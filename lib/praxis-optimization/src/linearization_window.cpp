#include "praxis/optimization/linearization_window.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <imgui.h>

#include <format>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::optimization {

namespace {

constexpr const char *nothing_published = "The function has not been evaluated yet.";

// g = -e and g' = J, each a single entry for a function of one variable.
expected<double, refusal> value_of(const scenario_linearization &at)
{
    if(!at.error)
        return unexpected(at.error.error());

    return at.error->size() == 1 ? expected<double, refusal>(-(*at.error)[0]) : unexpected(refusal::unsupported_input);
}

expected<double, refusal> slope_of(const scenario_linearization &at)
{
    if(!at.jacobian)
        return unexpected(at.jacobian.error());

    return at.jacobian->size() == 1 ? expected<double, refusal>((*at.jacobian)(0, 0)) : unexpected(refusal::unsupported_input);
}

std::string written(const char *named, const expected<double, refusal> &answered)
{
    return answered ? std::format("{} = {:.10g}", named, *answered) : std::format("{}: refused, {}", named, refusal_text(answered.error()));
}

}

std::vector<std::string> linearization_lines(const scenario_linearization &at, bool checks_derivative)
{
    std::vector<std::string> lines{written("g(x)", value_of(at)), written("g'(x)", slope_of(at))};
    if(!checks_derivative || !at.check || at.check->central.size() != 1)
        return lines;

    lines.push_back(std::format("g'(x) by central difference = {:.10g}", at.check->central(0, 0)));
    if(at.check->disagrees)
        lines.push_back(std::format("The derivative handed in and the central difference differ at x = {:.10g} by a relative {:.2e}.", at.point[0], at.check->difference));

    return lines;
}

linearization_window::linearization_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_checks(inputs.checks_derivative)
        , m_point(Eigen::VectorXd::Constant(1, (inputs.opened.ranges.front().lower + inputs.opened.ranges.front().upper) / 2.0))
        , m_seen(inputs.seen)
        , m_scenario(inputs.scenario)
{
    post();
}

void linearization_window::render()
{
    const std::shared_ptr<const scenario_snapshot> shown = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(shown == nullptr)
    {
        ImGui::TextUnformatted(nothing_published);
        ImGui::End();
        return;
    }

    const bool typed = ImGui::InputDouble("Point x", &m_point[0], 0.0, 0.0, "%.10g");
    const bool slid  = ImGui::SliderScalar("##along the range", ImGuiDataType_Double, &m_point[0], &shown->ranges.front().lower, &shown->ranges.front().upper, "%.6g");
    if(typed || slid)
        post();

    if(shown->linearization)
        for(const std::string &line : linearization_lines(*shown->linearization, m_checks))
            ImGui::TextWrapped("%s", line.c_str());
    ImGui::End();
}

void linearization_window::post() const
{
    command(m_scenario, [point = m_point](scenario_state &state) { state.linearize(point); });
}

}
