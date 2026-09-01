#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_MACHINE_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_MACHINE_H

#include "demo_documents.h"

#include "praxis/presets/arm.h"

#include "praxis/config/error.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <cstddef>
#include <optional>
#include <filesystem>

namespace praxis::demo {

// How many scenarios a document may name, which is one per composer the registration can reach. The
// spelling table and the composer table are in two translation units, and this is what holds each of
// them to the same length as the other.
constexpr std::size_t scenario_count = 10;

// The values one preset composes from, at the root of a space of its own, so a document read
// against another one is refused rather than answered from fallbacks throughout.
config::declaration machine_keyspace();

// The spellings a document may name a scenario by, in the order of the composer table they index.
std::span<const char *const> arm_scenario_labels();

// The description a preset's own document names, written package-relative, and nothing where it
// names none.
std::string machine_description(const config::document &values);

// The name a preset is shown under, stated by its own document, and nothing where it states none.
std::string preset_name(const config::document &values);

// Which of the spellings above a document names, as its place in that table.
std::size_t preset_scenario(const config::document &values);

// One preset's binding and the document it was read through, so a caller that has checked a preset
// reads its name and its scenario without loading the document a second time.
struct offered_document
{
    config::binding bound;
    config::document carried;
};

// One named document read against the machine keyspace. A name has two candidate places and which
// of them answers depends on a file a save creates, so the location is derived from the name each
// time it is wanted rather than held from an earlier time.
config::binding machine_binding(const std::filesystem::path &named, const documents &mine);

// The application names each preset's document, and the resolver answers where it is read from --
// this application's own copy where it has one, the shipped document otherwise. The library is
// handed a location and searches for nothing.
expected<config::binding, config::error> preset_binding(const config::document &values, const std::string &key, const documents &mine);

// The binding a preset composes through, and nothing where it is not offered: a preset the
// application's document names no document for, or whose document names no description, is left out
// and named.
std::optional<offered_document> offered_preset(const config::document &values, const std::string &key, const documents &mine);

// Where one machine keeps a chain somebody supplied: this application's own copy, made from the
// shipped document where there is one, since the control that keeps a chain saves through this
// binding. A machine naming no such document keeps nothing, and the scenario reading this opens at
// a chain nobody supplied.
std::optional<config::binding> machine_screw_table(const config::document &values, const documents &mine);

// The keys the windows an arm scenario opens carry, declared beneath the paths `window_paths` names
// and read back out from under those same ones, so a window's declaration and its reading stand
// beside each other. A start of a width other than `joints` is declined by name, a document carrying
// no joint count of its own.
void declare_windows(config::declaration &shape);
void read_windows(presets::arm_scenario &read, const config::document &values, std::size_t joints);

// A description path is written package-relative and resolved against `package_root` here.
presets::arm_scenario read_machine(const config::document &values, const std::filesystem::path &package_root);

}

#endif
