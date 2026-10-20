#include "optimization_keys.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/scenario_state.h"
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

constexpr const char *function_name_key = "function/name";
constexpr const char *limit_key         = "solve/limit";
constexpr const char *step_key          = "solve/step";

constexpr std::array<const char *, 2> lower_keys{"range/x_lower", "range/y_lower"};
constexpr std::array<const char *, 2> upper_keys{"range/x_upper", "range/y_upper"};
constexpr std::array<const char *, 2> seed_keys{"seed/x", "seed/y"};

void declare_preset(config::declaration &shape)
{
    const std::span<const char *const> labels = optimization_scenario_labels();

    shape.group("preset");
    shape.field(keys::preset_name, config::field_kind::text, std::string());
    shape.choice(keys::preset_scenario, std::vector<std::string>(labels.begin(), labels.end()), labels.front());
    shape.group("function");
    shape.field(function_name_key, config::field_kind::text, std::string());
}

void declare_solve(config::declaration &shape)
{
    shape.group("range");
    shape.field(lower_keys[0], config::field_kind::real, "-3");
    shape.field(upper_keys[0], config::field_kind::real, "3");
    shape.field(lower_keys[1], config::field_kind::real, "-3");
    shape.field(upper_keys[1], config::field_kind::real, "3");
    shape.group("seed");
    shape.field(seed_keys[0], config::field_kind::real, "0.75");
    shape.field(seed_keys[1], config::field_kind::real, "1");
    shape.group("solve");
    shape.field(limit_key, config::field_kind::integer, "200");
    shape.field(step_key, config::field_kind::integer, "0");
}

std::string selected_name(const config::document &values)
{
    return values.text(function_name_key).value_or(std::string());
}

double real_at(const config::document &values, const char *key)
{
    const expected<double, config::error> read = values.real(key);

    return read ? read.value() : std::numeric_limits<double>::quiet_NaN();
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
                                           count_at(values, limit_key),
                                           count_at(values, step_key)};
    for(std::size_t at = 0; at < dimension && at < seed_keys.size(); ++at)
    {
        opened.ranges.push_back(optimization::interval{real_at(values, lower_keys[at]), real_at(values, upper_keys[at])});
        opened.seed[static_cast<Eigen::Index>(at)] = real_at(values, seed_keys[at]);
    }

    return opened;
}

}
