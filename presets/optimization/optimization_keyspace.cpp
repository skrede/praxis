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

#include <span>
#include <limits>
#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <filesystem>

namespace praxis::presets {

namespace {

constexpr const char *function_name_key = "function/name";
constexpr const char *x_lower_key       = "range/x_lower";
constexpr const char *x_upper_key       = "range/x_upper";
constexpr const char *seed_x_key        = "seed/x";
constexpr const char *limit_key         = "solve/limit";

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
    shape.field(x_lower_key, config::field_kind::real, "-4");
    shape.field(x_upper_key, config::field_kind::real, "4");
    shape.field("range/y_lower", config::field_kind::real, "-4");
    shape.field("range/y_upper", config::field_kind::real, "4");
    shape.group("seed");
    shape.field(seed_x_key, config::field_kind::real, "1");
    shape.field("seed/y", config::field_kind::real, "1");
    shape.group("solve");
    shape.field(limit_key, config::field_kind::integer, "70");
    shape.field("solve/step", config::field_kind::integer, "0");
}

std::string selected_name(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(function_name_key);

    return read ? read.value() : std::string();
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

optimization::scenario_settings keys::curve_settings(const config::document &values, double tolerance)
{
    const optimization::interval range{real_at(values, x_lower_key), real_at(values, x_upper_key)};

    return optimization::scenario_settings{selected_name(values), range, real_at(values, seed_x_key), count_at(values, limit_key), tolerance};
}

}
