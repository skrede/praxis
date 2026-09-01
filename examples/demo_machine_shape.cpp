#include "demo_machine.h"

#include "praxis/rigid_motion/angles.h"

#include <map>
#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::demo {

namespace {

// All three tables are in their enumeration's own order, which is what reading one back as an index
// and casting relies on. The scenario spellings index the composer table the registration walks, so
// their order is that table's order and their number is that table's number.
constexpr std::array<const char *, 3> evaluation_policies{"fail", "warn", "skip"};
constexpr std::array<const char *, 3> missing_asset_policies{"fail", "warn", "skip"};
constexpr std::array scenario_spellings{
        "every window",          "forward kinematics", "supplied chain",  "tool and world object", "numerical inverse kinematics", "analytic inverse kinematics",
        "point to point motion", "via point motion",   "path comparison", "velocity kinematics"};

static_assert(scenario_spellings.size() == scenario_count);

std::string under(std::string_view at, std::string_view leaf)
{
    return std::string(at) + "/" + std::string(leaf);
}

std::vector<std::string> spelled(std::span<const char *const> labels)
{
    return std::vector<std::string>(labels.begin(), labels.end());
}

std::string text_at(const config::document &values, const std::string &key)
{
    const expected<std::string, config::error> read = values.text(key);

    return read ? read.value() : std::string();
}

double real_at(const config::document &values, const std::string &key)
{
    const expected<double, config::error> read = values.real(key);

    return read ? read.value() : 0.0;
}

std::size_t indexed(const config::document &values, const std::string &key, std::span<const char *const> labels)
{
    const std::string read = text_at(values, key);
    for(std::size_t option = 0u; option < labels.size(); ++option)
        if(read == labels[option])
            return option;

    return 0u;
}

std::string keyed(const config::document &values, const std::string &collection, const std::string &identity, std::string_view leaf)
{
    const expected<std::string, config::error> addressed = values.key(collection, identity, leaf);

    return addressed ? addressed.value() : std::string();
}

void declare_description(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    shape.field(under(at, "path"), config::field_kind::text, "");
    shape.choice(under(at, "evaluation"), spelled(evaluation_policies), evaluation_policies[0]);
    shape.choice(under(at, "missing_asset"), spelled(missing_asset_policies), missing_asset_policies[0]);
    shape.collection(under(at, "argument"), "index");
    shape.field(under(at, "argument/name"), config::field_kind::text, "");
    shape.field(under(at, "argument/value"), config::field_kind::text, "");
}

void declare_preset(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    shape.field(under(at, "name"), config::field_kind::text, "");
    shape.choice(under(at, "scenario"), spelled(scenario_spellings), scenario_spellings[0]);
}

void declare_initial(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    shape.collection(under(at, "joint"), "index");
    shape.field(under(at, "joint/degrees"), config::field_kind::real, "0");
}

std::map<std::string, std::string> read_arguments(const config::document &values, const std::string &at)
{
    std::map<std::string, std::string> named;
    for(const std::string &instance : values.identities(at))
        named.emplace(text_at(values, keyed(values, at, instance, "name")), text_at(values, keyed(values, at, instance, "value")));

    return named;
}

meios::load_options read_options(const config::document &values, const std::string &at, const std::filesystem::path &package_root)
{
    meios::load_options options;
    options.package_roots.push_back(package_root);
    options.eval       = static_cast<meios::eval_policy>(indexed(values, under(at, "evaluation"), evaluation_policies));
    options.on_missing = static_cast<meios::missing_asset>(indexed(values, under(at, "missing_asset"), missing_asset_policies));
    options.args       = read_arguments(values, under(at, "argument"));

    return options;
}

// The joint values are in the order the document carries them, which is the order the axes are in.
manipulator::joint_vector read_initial(const config::document &values, const std::string &at)
{
    const std::vector<std::string> present = values.identities(at);

    manipulator::joint_vector initial(static_cast<Eigen::Index>(present.size()));
    for(std::size_t axis = 0u; axis < present.size(); ++axis)
        initial[static_cast<Eigen::Index>(axis)] = real_at(values, keyed(values, at, present[axis], "degrees")) * radians_per_degree;

    return initial;
}

}

config::declaration machine_keyspace()
{
    config::declaration shape("machine");
    declare_preset(shape, "preset");
    declare_description(shape, "description");
    declare_initial(shape, "initial");
    declare_windows(shape);
    shape.group("screw_table");
    shape.field("screw_table/document", config::field_kind::text, "");

    return shape;
}

std::span<const char *const> arm_scenario_labels()
{
    return scenario_spellings;
}

std::string machine_description(const config::document &values)
{
    return text_at(values, "description/path");
}

std::string preset_name(const config::document &values)
{
    return text_at(values, "preset/name");
}

std::size_t preset_scenario(const config::document &values)
{
    return indexed(values, "preset/scenario", scenario_spellings);
}

presets::arm_scenario read_machine(const config::document &values, const std::filesystem::path &package_root)
{
    presets::arm_scenario read;
    read.options     = read_options(values, "description", package_root);
    read.description = package_root / machine_description(values);
    read.initial     = read_initial(values, "initial/joint");
    read_windows(read, values, static_cast<std::size_t>(read.initial.size()));

    return read;
}

}
