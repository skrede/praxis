#include "configuration_keys.h"

#include "praxis/manipulator/trajectory_configuration.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <array>
#include <locale>
#include <string>
#include <vector>
#include <cstddef>
#include <sstream>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct comparison_names
{
    static constexpr std::string_view first       = "first";
    static constexpr std::string_view second      = "second";
    static constexpr std::string_view screw       = "screw";
    static constexpr std::string_view played      = "played";
    static constexpr std::string_view decoupled   = "decoupled";
    static constexpr std::string_view joint_space = "joint_space";
};

// The values an end carries, in radians and separated by single spaces. The stream is imbued with
// the classic locale because a decimal point is a property of the document's grammar rather than of
// whatever locale the process happens to run under.
std::optional<joint_vector> values_of(const std::string &text)
{
    std::istringstream reader(text);
    reader.imbue(std::locale::classic());

    std::vector<double> taken;
    for(double value = 0.0; reader >> value;)
        taken.push_back(value);
    if(!reader.eof() || taken.empty())
        return std::nullopt;

    return joint_vector(Eigen::Map<const Eigen::VectorXd>(taken.data(), static_cast<Eigen::Index>(taken.size())));
}

std::string joined(const joint_vector &end)
{
    std::string text;
    for(Eigen::Index joint = 0; joint < end.size(); ++joint)
    {
        if(joint != 0)
            text += ' ';
        text += config::exact_text(end[joint]);
    }

    return text;
}

// A document naming no end at all leaves the window opening at the pair its own opening answers, so
// an absent value is the absence of a choice rather than an end of no joints.
joint_vector end_at(const config::document &values, std::string_view at, std::string_view leaf, std::size_t joints)
{
    const std::string carried = keys::text_at(values, keys::under(at, leaf), std::string_view());
    if(carried.empty())
        return joint_vector();

    const std::optional<joint_vector> read = values_of(carried);
    if(read && read->size() == static_cast<Eigen::Index>(joints))
        return *read;

    spdlog::error("praxis: 'manipulator.read_path_comparison' was given a {} end of {} joint values for an arm of {} joints, so it was not read and the end beside it still is", leaf,
                  read ? read->size() : 0, joints);

    return joint_vector();
}

// In the enumeration's own order, which is what reading one back as an index and casting relies on.
constexpr std::array<const char *, 3> played_spellings{"joint space", "decoupled", "screw"};

}

void declare_path_comparison(config::declaration &shape, std::string_view at)
{
    const path_comparison_window::settings opened;

    shape.group(std::string(at));
    shape.field(keys::under(at, comparison_names::first), config::field_kind::text, std::string());
    shape.field(keys::under(at, comparison_names::second), config::field_kind::text, std::string());
    shape.field(keys::under(at, comparison_names::joint_space), config::field_kind::flag, opened.joint_space ? "true" : "false");
    shape.field(keys::under(at, comparison_names::decoupled), config::field_kind::flag, opened.decoupled ? "true" : "false");
    shape.field(keys::under(at, comparison_names::screw), config::field_kind::flag, opened.screw ? "true" : "false");
    shape.choice(keys::under(at, comparison_names::played), keys::spelled(played_spellings), played_spellings[static_cast<std::size_t>(opened.played)]);
}

path_comparison_window::settings read_path_comparison(const config::document &values, std::string_view at, std::size_t joints)
{
    const path_comparison_window::settings opened;
    const std::size_t chosen = keys::indexed(values, keys::under(at, comparison_names::played), played_spellings, static_cast<std::size_t>(opened.played));

    return path_comparison_window::settings{end_at(values, at, comparison_names::first, joints),
                                            end_at(values, at, comparison_names::second, joints),
                                            keys::flag_at(values, keys::under(at, comparison_names::joint_space), opened.joint_space),
                                            keys::flag_at(values, keys::under(at, comparison_names::decoupled), opened.decoupled),
                                            keys::flag_at(values, keys::under(at, comparison_names::screw), opened.screw),
                                            static_cast<compared_path>(chosen)};
}

std::vector<config::edit> write_path_comparison(const path_comparison_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{config::edit{keys::under(at, comparison_names::first), joined(state.first)},
                                     config::edit{keys::under(at, comparison_names::second), joined(state.second)},
                                     config::edit{keys::under(at, comparison_names::joint_space), state.joint_space ? "true" : "false"},
                                     config::edit{keys::under(at, comparison_names::decoupled), state.decoupled ? "true" : "false"},
                                     config::edit{keys::under(at, comparison_names::screw), state.screw ? "true" : "false"},
                                     config::edit{keys::under(at, comparison_names::played), played_spellings[static_cast<std::size_t>(state.played)]}};
}

}
