#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_REGISTRATION_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_REGISTRATION_H

#include "praxis/presets/arm.h"
#include "praxis/presets/routes.h"
#include "praxis/presets/arm_scenarios.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset_registry.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace praxis::presets {

config::declaration arm_keyspace();

std::span<const char *const> arm_scenario_labels();

config::binding arm_binding(const std::filesystem::path &named, const std::filesystem::path &beside);

arm_scenario read_arm(const config::document &values, std::span<const std::filesystem::path> roots);

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced);

std::vector<std::string> register_arms(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents,
                                       std::span<const std::filesystem::path> roots, document_route located, composed_route announced, const manipulator::capabilities &arm,
                                       const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions);

}

#endif
