#include "configuration_keys.h"

#include "praxis/manipulator/kinematics_configuration.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

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

struct seed_names
{
    static constexpr std::string_view start  = "start";
    static constexpr std::string_view index  = "index";
    static constexpr std::string_view joints = "joints";
};

// The values a row carries, in radians and separated by single spaces. The stream is imbued with
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

std::string joined(const joint_vector &seed)
{
    std::string text;
    for(Eigen::Index joint = 0; joint < seed.size(); ++joint)
    {
        if(joint != 0)
            text += ' ';
        text += config::exact_text(seed[joint]);
    }

    return text;
}

// A row carrying no value at all is a row a shorter list left behind, so it is passed over rather
// than read as a start of no joints.
std::string row_text(const config::document &values, const std::string &rows, const std::string &identity)
{
    const expected<std::string, config::error> key = values.key(rows, identity, seed_names::joints);
    if(!key)
        return std::string();

    const expected<std::string, config::error> text = values.text(*key);

    return text ? *text : std::string();
}

}

void declare_ik_seeds(config::declaration &shape, std::string_view at)
{
    const std::string rows = keys::under(at, seed_names::start);

    shape.group(std::string(at));
    shape.collection(rows, std::string(seed_names::index));
    shape.field(keys::under(rows, seed_names::joints), config::field_kind::text, std::string());
}

ik_seed_window::settings read_ik_seeds(const config::document &values, std::string_view at, std::size_t joints)
{
    const std::string rows = keys::under(at, seed_names::start);

    std::vector<joint_vector> seeds;
    for(const std::string &identity : values.identities(rows))
    {
        const std::string carried = row_text(values, rows, identity);
        if(carried.empty())
            continue;

        const std::optional<joint_vector> read = values_of(carried);
        if(read && read->size() == static_cast<Eigen::Index>(joints))
        {
            seeds.push_back(*read);
            continue;
        }

        spdlog::error("praxis: 'manipulator.read_ik_seeds' was given a start of {} joint values at row {} for an arm of {} joints, so it was not read into the list and the starts "
                      "beside it still are",
                      read ? read->size() : 0, identity, joints);
    }

    return ik_seed_window::settings{std::move(seeds)};
}

std::vector<config::edit> write_ik_seeds(const config::document &values, const ik_seed_window::settings &state, std::string_view at)
{
    const std::string rows    = keys::under(at, seed_names::start);
    const std::size_t carried = values.identities(rows).size();

    std::vector<config::edit> changes;
    for(std::size_t row = 0; row < std::max(carried, state.seeds.size()); ++row)
    {
        const std::string where = rows + "[" + std::to_string(row) + "]";
        if(row >= carried)
            changes.push_back(config::edit{keys::under(where, seed_names::index), std::to_string(row + 1u)});

        changes.push_back(config::edit{keys::under(where, seed_names::joints), row < state.seeds.size() ? joined(state.seeds[row]) : std::string()});
    }

    return changes;
}

}
