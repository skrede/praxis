#include "scenario_answers.h"

#include "praxis/optimization/linearization_window.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::optimization {

namespace {

constexpr std::array<const char *, 2> point_labels{"Point x", "Point y"};
constexpr std::array<const char *, 2> slider_labels{"##along the range of x", "##along the range of y"};

// The value is -e and the slope J, a single entry for a function of one variable and a row of two for
// one of two.
expected<Eigen::VectorXd, refusal> value_of(const scenario_linearization &at)
{
    if(!at.error)
        return unexpected(at.error.error());

    return at.error->size() == 1 ? expected<Eigen::VectorXd, refusal>(-*at.error) : unexpected(refusal::unsupported_input);
}

expected<Eigen::VectorXd, refusal> slope_of(const scenario_linearization &at)
{
    if(!at.jacobian)
        return unexpected(at.jacobian.error());

    return at.jacobian->rows() == 1 && at.jacobian->cols() == at.point.size() ? expected<Eigen::VectorXd, refusal>(at.jacobian->row(0).transpose())
                                                                              : unexpected(refusal::unsupported_input);
}

std::string components(const Eigen::VectorXd &v)
{
    if(v.size() == 1)
        return std::format("{:.10g}", v[0]);

    std::string joined;
    for(Eigen::Index i = 0; i < v.size(); ++i)
        joined += (i == 0 ? "(" : ", ") + std::format("{:.10g}", v[i]);

    return joined + ")";
}

std::string written(const char *named, const expected<Eigen::VectorXd, refusal> &answered)
{
    return answered ? std::format("{} = {}", named, components(*answered)) : std::format("{}: refused, {}", named, refusal_text(answered.error()));
}

struct naming
{
    const char *value;
    const char *slope;
    const char *slope_handed_in;
    const char *at;
};

Eigen::VectorXd middle_of(const std::vector<interval> &ranges)
{
    Eigen::VectorXd middle(static_cast<Eigen::Index>(ranges.size()));
    for(std::size_t axis = 0; axis < ranges.size(); ++axis)
        middle[static_cast<Eigen::Index>(axis)] = (ranges[axis].lower + ranges[axis].upper) / 2.0;

    return middle;
}

naming named_for(Eigen::Index dimension)
{
    if(dimension == 1)
        return naming{"g(x)", "g'(x)", "The derivative", "x"};

    return naming{"f(x, y)", "grad f(x, y)", "The gradient", "(x, y)"};
}

}

std::vector<std::string> linearization_lines(const scenario_linearization &at, bool checks_derivative)
{
    const naming names = named_for(at.point.size());
    std::vector<std::string> lines{written(names.value, value_of(at)), written(names.slope, slope_of(at))};
    if(!checks_derivative || !at.check || at.check->central.rows() != 1 || at.check->central.cols() != at.point.size())
        return lines;

    const Eigen::VectorXd central = at.check->central.row(0).transpose();
    lines.push_back(std::format("{} by central difference = {}", names.slope, components(central)));
    if(at.check->disagrees)
        lines.push_back(std::format("{} handed in and the central difference differ at {} = {} by a relative {:.2e}.", names.slope_handed_in, names.at, components(at.point),
                                    at.check->difference));

    return lines;
}

linearization_window::linearization_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_checks(inputs.checks_derivative)
        , m_point(middle_of(inputs.opened.ranges))
        , m_seen(inputs.seen)
        , m_drawn(inputs.drawn != nullptr ? inputs.drawn : std::make_shared<scenario_view>())
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

    controls(shown->ranges);
    if(shown->linearization)
        for(const std::string &line : linearization_lines(*shown->linearization, m_checks))
            ImGui::TextWrapped("%s", line.c_str());
    ImGui::End();
}

void linearization_window::controls(const std::vector<interval> &ranges)
{
    if(edit_point(ranges))
        post();
    if(ImGui::Button("Reset"))
    {
        m_point = middle_of(ranges);
        post();
    }
    ImGui::Checkbox("Show", &m_drawn->linearization);
}

bool linearization_window::edit_point(const std::vector<interval> &ranges)
{
    bool edited = false;
    for(std::size_t axis = 0; axis < point_labels.size() && axis < ranges.size() && axis < static_cast<std::size_t>(m_point.size()); ++axis)
    {
        double &coordinate = m_point[static_cast<Eigen::Index>(axis)];
        ImGui::InputDouble(point_labels[axis], &coordinate, 0.0, 0.0, "%.10g");
        edited |= ImGui::IsItemDeactivatedAfterEdit();
        edited |= ImGui::SliderScalar(slider_labels[axis], ImGuiDataType_Double, &coordinate, &ranges[axis].lower, &ranges[axis].upper, "%.6g");
    }

    return edited;
}

void linearization_window::post() const
{
    command(m_scenario, [point = m_point](scenario_state &state) { state.linearize(point); });
}

}
