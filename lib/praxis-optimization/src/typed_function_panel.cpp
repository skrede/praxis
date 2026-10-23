#include "typed_function_panel.h"

#include "typed/parse.h"
#include "typed/expression.h"

#include "praxis/optimization/settings_keys.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <imgui.h>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <string_view>

namespace praxis::optimization {

namespace {

constexpr const char *failure_marker = ">>";

std::array<char, typed::expression_capacity + 1> holding(const std::string &text)
{
    std::array<char, typed::expression_capacity + 1> held{};
    std::copy_n(text.begin(), std::min(text.size(), typed::expression_capacity), held.begin());

    return held;
}

void render_reading(std::string_view text, const expected<typed::expression, typed::parse_failure> &read)
{
    if(read)
    {
        ImGui::TextUnformatted(typed::echoed(*read).c_str());
        return;
    }

    const std::size_t at     = std::min(read.error().offset, text.size());
    const std::string marked = std::format("{}{}{}", text.substr(0, at), failure_marker, text.substr(at));
    ImGui::TextUnformatted(marked.c_str());
    ImGui::TextUnformatted(std::format("At character {}: expected {}", read.error().offset + 1u, read.error().expected).c_str());
}

std::vector<config::edit> leaf_edits(const std::string &instance, const typed_parameter &parameter)
{
    return {config::edit(instance + "/" + parameter_value_leaf, config::exact_text(parameter.value)),
            config::edit(instance + "/" + parameter_lower_leaf, config::exact_text(parameter.range.lower)),
            config::edit(instance + "/" + parameter_upper_leaf, config::exact_text(parameter.range.upper))};
}

// An instance the document does not carry is named ahead of its values, at the ordinal `appending`
// stands at, which then moves on.
void append_parameter(std::vector<config::edit> &edits, const config::document &carried, const typed_parameter &parameter, std::size_t &appending)
{
    const std::string name(1u, parameter.name);
    const expected<std::string, config::error> carried_value = carried.key(parameters_key, name, parameter_value_leaf);
    if(carried_value)
    {
        const std::string instance              = carried_value->substr(0, carried_value->rfind('/'));
        const std::vector<config::edit> unsaved = config::unsaved_edits(carried, leaf_edits(instance, parameter));
        edits.insert(edits.end(), unsaved.begin(), unsaved.end());
        return;
    }

    const std::string instance = std::format("{}[{}]", parameters_key, appending++);
    edits.emplace_back(instance + "/" + parameter_name_leaf, name);
    const std::vector<config::edit> values = leaf_edits(instance, parameter);
    edits.insert(edits.end(), values.begin(), values.end());
}

}

typed_function_panel::typed_function_panel(const scenario_window_inputs &inputs)
        : m_dimension(inputs.dimension)
        , m_asked(inputs.opened.typed.expression)
        , m_rows(inputs.seen, inputs.scenario)
        , m_scenario(inputs.scenario)
        , m_text(holding(inputs.opened.typed.expression))
{
}

void typed_function_panel::render()
{
    ImGui::TextUnformatted(m_dimension == 1u ? "f(x) =" : "f(x, y) =");
    ImGui::SameLine();
    const bool entered = ImGui::InputText("##expression", m_text.data(), m_text.size(), ImGuiInputTextFlags_EnterReturnsTrue);
    const bool left    = ImGui::IsItemDeactivatedAfterEdit();

    const std::string_view text(m_text.data());
    const expected<typed::expression, typed::parse_failure> read = typed::parsed(text, m_dimension);
    render_reading(text, read);
    if((entered || left) && read && text != m_asked)
        post(std::string(text));
    m_rows.render();
}

const std::string &typed_function_panel::asked() const
{
    return m_asked;
}

std::vector<config::edit> typed_function_panel::settings_edits(const config::document &carried) const
{
    std::vector<config::edit> edits                              = config::unsaved_edits(carried, std::vector<config::edit>{config::edit(expression_key, m_asked)});
    const expected<typed::expression, typed::parse_failure> read = typed::parsed(m_asked, m_dimension);
    const auto used                                              = [&read](char letter) { return !read || read->parameters.find(letter) != std::string::npos; };

    const std::vector<std::string> identities = carried.identities(parameters_key);
    std::size_t appending                     = identities.size();
    for(const typed_parameter &parameter : m_rows.held())
        if(used(parameter.name))
            append_parameter(edits, carried, parameter, appending);

    if(!read)
        return edits;

    std::vector<std::string> taken;
    for(const std::string &identity : identities)
        if(!identity.empty() && !(identity.size() == 1u && used(identity.front())) && std::ranges::find(taken, identity) == taken.end())
            taken.push_back(identity);
    for(const std::string &identity : taken)
        edits.emplace_back(parameters_key, identity, config::edit_kind::taken_out);

    return edits;
}

void typed_function_panel::post(std::string text)
{
    m_asked = text;
    command(m_scenario, [text = std::move(text)](scenario_state &state) mutable { state.set_expression(std::move(text)); });
}

}
