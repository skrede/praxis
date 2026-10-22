#include "scenario_defaults.h"
#include "scenario_admission.h"

#include "typed/parse.h"
#include "typed/evaluate.h"
#include "typed/expression.h"
#include "typed/typed_problem.h"

#include "praxis/optimization/scenario_state.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::optimization {

namespace {

std::optional<typed::expression> read_typed(const std::string &text, std::size_t dimension, const char *otherwise)
{
    expected<typed::expression, typed::parse_failure> read = typed::parsed(text, dimension);
    if(read)
        return std::move(*read);

    spdlog::error("praxis: '{}' does not read as a function of {}, since {} was expected at byte {}; {}", text, dimension == 1u ? "x" : "x and y", read.error().expected,
                  read.error().offset, otherwise);

    return std::nullopt;
}

typed::parameter_values values_of(const std::vector<typed_parameter> &parameters)
{
    typed::parameter_values values;
    values.fill(std::numeric_limits<double>::quiet_NaN());
    for(const typed_parameter &parameter : parameters)
        if(typed::lowercase(parameter.name))
            values[static_cast<std::size_t>(parameter.name - 'a')] = parameter.value;

    return values;
}

bool holds_parameter(const std::vector<typed_parameter> &parameters, char name)
{
    if(std::ranges::find(parameters, name, &typed_parameter::name) != parameters.end())
        return true;

    spdlog::error("praxis: the typed function uses no parameter '{}'; the last run stays shown", name);

    return false;
}

bool valued(char name, double value)
{
    if(std::isfinite(value))
        return true;

    spdlog::error("praxis: parameter '{}' takes a finite value, which {} is not; the last run stays shown", name, value);

    return false;
}

bool ranged(char name, const interval &range)
{
    if(std::isfinite(range.lower) && std::isfinite(range.upper) && range.lower < range.upper)
        return true;

    spdlog::error("praxis: parameter '{}' is chosen from a finite range whose minimum lies below its maximum, which [{}, {}] is not; the last run stays shown", name, range.lower,
                  range.upper);

    return false;
}

}

std::vector<typed_parameter> parameters_for(std::string_view letters, const std::vector<typed_parameter> &held)
{
    std::vector<typed_parameter> kept;
    for(const char letter : letters)
    {
        const auto found = std::ranges::find(held, letter, &typed_parameter::name);
        kept.push_back(found != held.end() ? *found : typed_parameter{letter, parameter_value, interval{parameter_lower, parameter_upper}});
    }

    return kept;
}

typed_settings lettered(typed_settings typed, std::size_t dimension)
{
    const expected<typed::expression, typed::parse_failure> read = typed::parsed(typed.expression, dimension);
    if(read)
        typed.parameters = parameters_for(read->parameters, typed.parameters);

    return typed;
}

void scenario_state::select_source(function_source source)
{
    mutate([source](scenario_settings &edited) { edited.typed.source = source; });
}

void scenario_state::set_expression(std::string text)
{
    const std::optional<typed::expression> read = read_typed(text, m_dimension, "the last run stays shown");
    if(!read)
        return;

    mutate(
            [&text, &read](scenario_settings &edited)
            {
                edited.typed.expression = std::move(text);
                edited.typed.parameters = parameters_for(read->parameters, edited.typed.parameters);
            });
}

void scenario_state::set_parameter(char name, double value)
{
    if(!holds_parameter(m_settings.typed.parameters, name) || !valued(name, value))
        return;

    std::ranges::find(m_settings.typed.parameters, name, &typed_parameter::name)->value = value;
    run();
}

void scenario_state::set_parameter_range(char name, interval range)
{
    if(!holds_parameter(m_settings.typed.parameters, name) || !ranged(name, range))
        return;

    std::ranges::find(m_settings.typed.parameters, name, &typed_parameter::name)->range = range;
    run();
}

// The typed function is read again for every solve, so its tree is built and evaluated on this
// strand alone.
std::optional<std::size_t> scenario_state::posed_selection()
{
    if(m_settings.typed.source != function_source::manual)
        return selected_among(m_names, m_settings.selected);

    const std::optional<typed::expression> read = read_typed(m_settings.typed.expression, m_dimension, "nothing typed is drawn or solved");
    if(!read)
        return std::nullopt;

    const auto tree                      = std::make_shared<const typed::expression>(*read);
    const typed::parameter_values values = values_of(m_settings.typed.parameters);
    m_problems.resize(m_names.size());
    m_objectives.resize(m_names.size());
    m_problems.push_back(typed::typed_residual(tree, values, m_dimension == 1u ? curve_tolerance : surface_tolerance));
    m_objectives.push_back(typed::typed_objective(tree, values, m_dimension == 1u ? curve_objective_tolerance : surface_objective_tolerance));

    return m_names.size();
}

bool scenario_state::checking(std::size_t selected) const
{
    return m_checks && selected < m_names.size();
}

}
