#include "optimization_keys.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/settings_keys.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <span>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <filesystem>

namespace praxis::presets {

namespace {

void declare_preset(config::declaration &shape)
{
    const std::span<const char *const> labels = optimization_scenario_labels();

    shape.group("preset");
    shape.field(keys::preset_name, config::field_kind::text, std::string());
    shape.choice(keys::preset_scenario, std::vector<std::string>(labels.begin(), labels.end()), labels.front());
    shape.group("function");
    shape.field(optimization::function_name_key, config::field_kind::text, std::string());
}

std::string under_parameter(const char *leaf)
{
    return std::string(optimization::parameters_key) + "/" + leaf;
}

void declare_typed(config::declaration &shape)
{
    shape.choice(optimization::source_key, std::vector<std::string>(optimization::source_spellings.begin(), optimization::source_spellings.end()),
                 optimization::source_spellings.front());
    shape.field(optimization::expression_key, config::field_kind::text, std::string());
    shape.group("parameters");
    shape.collection(optimization::parameters_key, optimization::parameter_name_leaf);
    shape.field(under_parameter(optimization::parameter_value_leaf), config::field_kind::real, "1");
    shape.field(under_parameter(optimization::parameter_lower_leaf), config::field_kind::real, "-5");
    shape.field(under_parameter(optimization::parameter_upper_leaf), config::field_kind::real, "5");
}

void declare_solve(config::declaration &shape)
{
    shape.group("range");
    shape.field(optimization::range_lower_keys[0], config::field_kind::real, "-3");
    shape.field(optimization::range_upper_keys[0], config::field_kind::real, "3");
    shape.field(optimization::range_lower_keys[1], config::field_kind::real, "-3");
    shape.field(optimization::range_upper_keys[1], config::field_kind::real, "3");
    shape.group("seed");
    shape.field(optimization::seed_keys[0], config::field_kind::real, "0.75");
    shape.field(optimization::seed_keys[1], config::field_kind::real, "1");
    shape.group("solve");
    shape.field(optimization::limit_key, config::field_kind::integer, "200");
    shape.field(optimization::step_key, config::field_kind::integer, "0");
    shape.choice(optimization::form_key, std::vector<std::string>(optimization::form_spellings.begin(), optimization::form_spellings.end()), optimization::form_spellings.front());
    shape.choice(optimization::method_key, std::vector<std::string>(optimization::method_spellings.begin(), optimization::method_spellings.end()),
                 optimization::method_spellings.front());
    shape.field(optimization::step_length_key, config::field_kind::real, "0.1");
}

std::string selected_name(const config::document &values)
{
    return values.text(optimization::function_name_key).value_or(std::string());
}

double real_at(const config::document &values, const char *key)
{
    const expected<double, config::error> read = values.real(key);

    return read ? read.value() : std::numeric_limits<double>::quiet_NaN();
}

// The index of the spelling the document carries at `key`, which the keyspace holds to the table.
template<std::size_t count>
std::size_t spelled_at(const config::document &values, const char *key, const std::array<const char *, count> &spellings)
{
    const std::string read = values.text(key).value_or(std::string());
    const auto found       = std::ranges::find(spellings, read);

    return found == spellings.end() ? 0u : static_cast<std::size_t>(found - spellings.begin());
}

optimization::objective_settings objective_at(const config::document &values)
{
    return optimization::objective_settings{static_cast<optimization::problem_form>(spelled_at(values, optimization::form_key, optimization::form_spellings)),
                                            static_cast<optimization::minimizer_slot>(spelled_at(values, optimization::method_key, optimization::method_spellings)),
                                            real_at(values, optimization::step_length_key)};
}

double parameter_at(const config::document &values, const std::string &name, const char *leaf)
{
    const expected<std::string, config::error> key = values.key(optimization::parameters_key, name, leaf);

    return key ? real_at(values, key->c_str()) : std::numeric_limits<double>::quiet_NaN();
}

// An instance whose name is not one lowercase letter names no parameter, so it is not read.
optimization::typed_settings typed_at(const config::document &values)
{
    optimization::typed_settings typed{static_cast<optimization::function_source>(spelled_at(values, optimization::source_key, optimization::source_spellings)),
                                       values.text(optimization::expression_key).value_or(std::string()),
                                       {}};
    for(const std::string &name : values.identities(optimization::parameters_key))
        if(name.size() == 1u && name.front() >= 'a' && name.front() <= 'z')
            typed.parameters.push_back(optimization::typed_parameter{
                    name.front(), parameter_at(values, name, optimization::parameter_value_leaf),
                    optimization::interval{parameter_at(values, name, optimization::parameter_lower_leaf), parameter_at(values, name, optimization::parameter_upper_leaf)}});
    std::ranges::sort(typed.parameters, {}, &optimization::typed_parameter::name);

    return typed;
}

std::uint32_t count_at(const config::document &values, const char *key)
{
    const expected<std::int64_t, config::error> read = values.integer(key);

    return read ? static_cast<std::uint32_t>(std::clamp<std::int64_t>(read.value(), 0, std::numeric_limits<std::uint32_t>::max())) : 0u;
}

}

config::declaration optimization_keyspace()
{
    config::declaration shape("optimization");
    declare_preset(shape);
    declare_typed(shape);
    declare_solve(shape);

    return shape;
}

config::binding optimization_binding(const std::filesystem::path &named, const std::filesystem::path &beside)
{
    return config::binding{optimization_keyspace(), config::resolve(named, beside), config::expectation::partial};
}

optimization::scenario_settings keys::opened_settings(const config::document &values, std::size_t dimension)
{
    optimization::scenario_settings opened{selected_name(values),
                                           {},
                                           Eigen::VectorXd::Constant(static_cast<Eigen::Index>(dimension), std::numeric_limits<double>::quiet_NaN()),
                                           count_at(values, optimization::limit_key),
                                           count_at(values, optimization::step_key),
                                           objective_at(values),
                                           typed_at(values)};
    for(std::size_t at = 0; at < dimension && at < optimization::seed_keys.size(); ++at)
    {
        opened.ranges.push_back(optimization::interval{real_at(values, optimization::range_lower_keys[at]), real_at(values, optimization::range_upper_keys[at])});
        opened.seed[static_cast<Eigen::Index>(at)] = real_at(values, optimization::seed_keys[at]);
    }

    return opened;
}

}
