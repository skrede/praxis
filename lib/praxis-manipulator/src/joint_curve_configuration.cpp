#include "configuration_keys.h"

#include "praxis/manipulator/trajectory_configuration.h"

#include <spdlog/spdlog.h>

#include <locale>
#include <string>
#include <vector>
#include <cstddef>
#include <sstream>
#include <utility>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct curve_names
{
    static constexpr std::string_view hidden = "hidden";
};

// The document counts the joints from one, as the names they are drawn under do, and the window
// from zero. Text carrying anything else is no list of joints at all rather than the prefix of one.
std::optional<std::vector<std::size_t>> counted_from_one(const std::string &text)
{
    std::istringstream reader(text);
    reader.imbue(std::locale::classic());

    std::vector<std::size_t> hidden;
    for(long long named = 0; reader >> named;)
    {
        if(named < 1)
            return std::nullopt;

        hidden.push_back(static_cast<std::size_t>(named - 1));
    }

    return reader.eof() ? std::optional<std::vector<std::size_t>>(std::move(hidden)) : std::nullopt;
}

std::vector<std::size_t> joints_of(const std::string &text)
{
    std::optional<std::vector<std::size_t>> read = counted_from_one(text);
    if(!read)
    {
        spdlog::error("praxis: 'manipulator.read_joint_curves' was given '{}', which names no joint counted from one, so every joint is left drawn", text);

        return {};
    }

    std::sort(read->begin(), read->end());
    read->erase(std::unique(read->begin(), read->end()), read->end());

    return std::move(*read);
}

std::string joined(const std::vector<std::size_t> &hidden)
{
    std::string text;
    for(std::size_t joint : hidden)
    {
        if(!text.empty())
            text += ' ';
        text += std::to_string(joint + 1u);
    }

    return text;
}

}

void declare_joint_curves(config::declaration &shape, std::string_view at)
{
    shape.group(std::string(at));
    shape.field(keys::under(at, curve_names::hidden), config::field_kind::text, std::string());
}

joint_curve_window::settings read_joint_curves(const config::document &values, std::string_view at)
{
    return joint_curve_window::settings{joints_of(keys::text_at(values, keys::under(at, curve_names::hidden), std::string_view()))};
}

std::vector<config::edit> write_joint_curves(const joint_curve_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{config::edit{keys::under(at, curve_names::hidden), joined(state.hidden)}};
}

}
