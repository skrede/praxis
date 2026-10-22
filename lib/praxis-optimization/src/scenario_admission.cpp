#include "registered_list.h"
#include "scenario_admission.h"

#include <spdlog/spdlog.h>

#include <span>
#include <cmath>
#include <format>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

const char *variables(std::size_t dimension)
{
    return dimension == 1u ? "one variable" : "two variables";
}

}

bool drawable(std::span<const interval> ranges, std::size_t dimension)
{
    return ranges.size() == dimension &&
            std::ranges::all_of(ranges, [](const interval &range) { return std::isfinite(range.lower) && std::isfinite(range.upper) && range.lower < range.upper; });
}

std::optional<std::size_t> selected_among(const std::vector<std::string> &names, const std::string &name)
{
    const std::optional<std::size_t> selected = index_of(names, name);
    if(!selected)
        spdlog::error("praxis: no function named '{}' is registered with this scenario; the last run stays shown", name);

    return selected;
}

bool drawn_over(std::span<const interval> ranges, std::size_t dimension)
{
    if(drawable(ranges, dimension))
        return true;

    spdlog::error("praxis: a function of {} is drawn over a finite range per variable whose minimum lies below its maximum; the last run stays shown", variables(dimension));

    return false;
}

bool taken_at(const Eigen::VectorXd &x, std::size_t dimension, const char *as)
{
    if(static_cast<std::size_t>(x.size()) == dimension && x.allFinite())
        return true;

    const std::string handed = static_cast<std::size_t>(x.size()) == dimension ? std::string("a value that is not finite") : std::format("a vector of {} values", x.size());
    spdlog::error("praxis: a function of {} is {}, which {} is not; the last run stays shown", variables(dimension), as, handed);

    return false;
}

bool admitted(const scenario_settings &edited, const std::vector<std::string> &names, std::size_t dimension)
{
    const bool named = edited.typed.source == function_source::manual || selected_among(names, edited.selected).has_value();

    return named && drawn_over(edited.ranges, dimension) && taken_at(edited.seed, dimension, "solved from one finite seed");
}

bool descends_by(double step_length)
{
    if(std::isfinite(step_length) && step_length > 0.0)
        return true;

    spdlog::error("praxis: gradient descent takes a finite, positive step length, which {} is not; the last run stays shown", step_length);

    return false;
}

}
