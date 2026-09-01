#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_CONFIGURATION_KEYS_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_CONFIGURATION_KEYS_H

#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/frame_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <Eigen/Core>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::rigid_motion::keys {

// Every key this mapping addresses is one of these names joined onto the caller's path, so a
// declared key and the edit that writes it cannot spell themselves differently.
struct arrangement_names
{
    static constexpr std::string_view name     = "name";
    static constexpr std::string_view euler    = "euler";
    static constexpr std::string_view order    = "euler_order";
    static constexpr std::string_view object   = "object";
    static constexpr std::string_view parent   = "parent";
    static constexpr std::string_view position = "position";
};

std::string under(std::string_view at, std::string_view leaf);

// The shortest text that reads back as the same single-precision value, so a component a person
// edits reads as they wrote it rather than as a widened decimal expansion.
std::string text_of(float value);

// The text `values` itself carries at `key`, and nothing where the value would come from the
// declaration instead, so a document saying nothing is told apart from one saying nothing is the value.
std::optional<std::string> carried_text(const config::document &values, const std::string &key);

void declare_vector(config::declaration &shape, const std::string &at, const Eigen::Vector3f &fallback);
Eigen::Vector3f read_vector(const config::document &values, const std::string &at, const Eigen::Vector3f &fallback);
void write_vector(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3f &value);

void declare_order(config::declaration &shape, const std::string &key, axis_order fallback);
axis_order read_order(const config::document &values, const std::string &key, axis_order fallback);
std::string order_text(axis_order value);

std::string name_of(std::optional<std::size_t> which, std::span<const std::string> objects);

// The instance of `collection` whose identity is `named`, addressed by the ordinal the document
// carries it at, or nothing where the document carries no such instance.
std::optional<std::string> instance_at(const config::document &values, const std::string &collection, const std::string &named);

// The leaves of `now` a save would carry differently from `was`, in the order a placement is
// written, and nothing at all where the two would be written the same, so an empty answer is what an
// unmoved placement is.
std::vector<config::edit> moved_leaves(const std::string &instance, const frame_window::placement &was, const frame_window::placement &now, std::span<const std::string> objects);

// The placement `state` carries at `which`, or a default-constructed one where it carries none
// there, so an opening shorter than the object list is read without indexing past its end.
frame_window::placement placed_at(const frame_window::settings &state, std::size_t which);

}

#endif
