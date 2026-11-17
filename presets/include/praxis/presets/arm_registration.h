#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_REGISTRATION_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_REGISTRATION_H

#include "praxis/presets/arm.h"
#include "praxis/presets/routes.h"
#include "praxis/presets/arm_scenarios.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset_registry.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <functional>

namespace praxis::presets {

config::declaration arm_keyspace();

std::span<const char *const> arm_scenario_labels();

config::binding arm_binding(const std::filesystem::path &named, const std::filesystem::path &beside);

arm_scenario read_arm(const config::document &values, std::span<const std::filesystem::path> roots);

struct arm_capabilities
{
    manipulator::capabilities arm;
    trajectory::capabilities shapes;
    rigid_motion::capabilities motions;
    optimization::capabilities solving;
};

// Asked once per offered document, at registration, for what the scenario that document names is
// composed with.
using capability_route = std::function<arm_capabilities(arm_scenario_kind)>;

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced);

// Each registered composer holds its own copy of the capabilities, so none of them need outlive this call.
std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced, const manipulator::capabilities &arm,
                                       const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions, const optimization::capabilities &solving);

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced, const capability_route &capabilities_for);

}

#endif
