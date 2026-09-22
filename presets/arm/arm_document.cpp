#include "arm_keys.h"
#include "arm_windows.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/types.h"

#include "praxis/rigid_motion/angles.h"

#include "praxis/config/document.h"

#include <meios/urdf/load.h>

#include <Eigen/Core>

#include <map>
#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <filesystem>

namespace praxis::presets {

namespace {

std::map<std::string, std::string> read_arguments(const config::document &values, const std::string &at)
{
    std::map<std::string, std::string> named;
    for(const std::string &instance : values.identities(at))
        named.emplace(keys::text_at(values, keys::keyed(values, at, instance, "name")), keys::text_at(values, keys::keyed(values, at, instance, "value")));

    return named;
}

meios::load_options read_options(const config::document &values, const std::string &at, std::span<const std::filesystem::path> roots)
{
    meios::load_options options;
    options.package_roots.assign(roots.begin(), roots.end());
    options.eval       = static_cast<meios::eval_policy>(keys::spelled_index(values, keys::under(at, "evaluation"), keys::evaluation_policies).value_or(0u));
    options.on_missing = static_cast<meios::missing_asset>(keys::spelled_index(values, keys::under(at, "missing_asset"), keys::missing_asset_policies).value_or(0u));
    options.args       = read_arguments(values, keys::under(at, "argument"));

    return options;
}

// The joint values are in the order the document carries them, which is the order the axes are in.
manipulator::joint_vector read_initial(const config::document &values, const std::string &at)
{
    const std::vector<std::string> present = values.identities(at);

    manipulator::joint_vector initial(static_cast<Eigen::Index>(present.size()));
    for(std::size_t axis = 0u; axis < present.size(); ++axis)
        initial[static_cast<Eigen::Index>(axis)] = keys::real_at(values, keys::keyed(values, at, present[axis], "degrees")) * radians_per_degree;

    return initial;
}

// The first supplied root holding the path answers. A path no root holds is carried as the document
// wrote it, so the failure named where the description is loaded is the spelling somebody typed
// rather than a place that was never chosen.
std::filesystem::path described_at(const std::filesystem::path &named, std::span<const std::filesystem::path> roots)
{
    for(const std::filesystem::path &root : roots)
        if(std::filesystem::exists(root / named))
            return root / named;

    return named;
}

// A model nobody named stays unnamed, since a root joined onto nothing is the root directory, which
// exists and would resolve to a place no mesh is loaded from.
std::string model_at(const std::string &named, std::span<const std::filesystem::path> roots)
{
    return named.empty() ? named : described_at(named, roots).string();
}

}

arm_scenario read_arm(const config::document &values, std::span<const std::filesystem::path> roots)
{
    arm_scenario read;
    read.options     = read_options(values, "description", roots);
    read.description = described_at(keys::text_at(values, keys::description_path_key), roots);
    read.initial     = read_initial(values, "initial/joint");
    read_arm_windows(read, values, static_cast<std::size_t>(read.initial.size()));

    read.tool.model_path         = model_at(read.tool.model_path, roots);
    read.world_object.model_path = model_at(read.world_object.model_path, roots);

    return read;
}

}
