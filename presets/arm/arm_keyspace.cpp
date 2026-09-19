#include "arm_keys.h"

#include "praxis/presets/arm_registration.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/declaration.h"

#include <span>
#include <string>
#include <filesystem>

namespace praxis::presets {

namespace {

void declare_description(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    shape.field(keys::under(at, "path"), config::field_kind::text, "");
    shape.choice(keys::under(at, "evaluation"), keys::spelled(keys::evaluation_policies), keys::evaluation_policies[0]);
    shape.choice(keys::under(at, "missing_asset"), keys::spelled(keys::missing_asset_policies), keys::missing_asset_policies[0]);
    shape.collection(keys::under(at, "argument"), "index");
    shape.field(keys::under(at, "argument/name"), config::field_kind::text, "");
    shape.field(keys::under(at, "argument/value"), config::field_kind::text, "");
}

void declare_preset(config::declaration &shape, const std::string &at)
{
    const std::span<const char *const> labels = arm_scenario_labels();

    shape.group(at);
    shape.field(keys::under(at, "name"), config::field_kind::text, "");
    shape.choice(keys::under(at, "scenario"), keys::spelled(labels), labels.front());
}

void declare_initial(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    shape.collection(keys::under(at, "joint"), "index");
    shape.field(keys::under(at, "joint/degrees"), config::field_kind::real, "0");
}

}

config::declaration arm_keyspace()
{
    config::declaration shape("arm");
    declare_preset(shape, "preset");
    declare_description(shape, "description");
    declare_initial(shape, "initial");
    shape.group("screw_table");
    shape.field(keys::screw_table_key, config::field_kind::text, "");

    return shape;
}

config::binding arm_binding(const std::filesystem::path &named, const std::filesystem::path &beside)
{
    return config::binding{arm_keyspace(), config::resolve(named, beside), config::expectation::partial};
}

}
