#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CONFIGURATION_KEYS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CONFIGURATION_KEYS_H

#include "praxis/manipulator/control_mode.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator::keys {

std::string under(std::string_view at, std::string_view leaf);

// The shortest text that reads back as the same single-precision value, so a component a person
// edits reads as they wrote it rather than as a widened decimal expansion.
std::string text_of(float value);

std::vector<std::string> spelled(std::span<const char *const> labels);

bool flag_at(const config::document &values, const std::string &key, bool fallback);
float real_at(const config::document &values, const std::string &key, float fallback);
std::string text_at(const config::document &values, const std::string &key, std::string_view fallback);

// The position of the document's value in `labels`, or `fallback` where the document carries none
// the list names.
std::size_t indexed(const config::document &values, const std::string &key, std::span<const char *const> labels, std::size_t fallback);

void declare_vector(config::declaration &shape, const std::string &at, const Eigen::Vector3f &fallback);
Eigen::Vector3f read_vector(const config::document &values, const std::string &at, const Eigen::Vector3f &fallback);
void write_vector(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3f &value);

// Every window carries the mode it commands under the same leaf name beneath its own key
// path, so what separates the groups is the path they were declared at and not the spelling of the
// leaf.
void declare_mode(config::declaration &shape, std::string_view at, control_mode fallback);
control_mode read_mode(const config::document &values, std::string_view at, control_mode fallback);
config::edit written_mode(control_mode mode, std::string_view at);

void declare_order(config::declaration &shape, const std::string &key, axis_order fallback);
axis_order read_order(const config::document &values, const std::string &key, axis_order fallback);
std::string order_text(axis_order value);

}

#endif
