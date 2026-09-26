#ifndef HPP_GUARD_PRAXIS_CONFIG_READDRESSING_H
#define HPP_GUARD_PRAXIS_CONFIG_READDRESSING_H

#include "praxis/config/writer.h"

#include <span>
#include <string>
#include <cstddef>
#include <optional>

namespace praxis::config {

// One instance a save takes out: the collection it hangs in as the save spelled it, which of that
// collection's instances it is, and the bytes it occupies.
struct taken
{
    std::string stem;
    std::size_t ordinal;
    std::size_t begin;
    std::size_t length;
};

// The ordinal `key` names among the instances of the collection at `stem`, read as any key segment
// is, so a segment with no bracket names the first; nothing where `key` does not pass through `stem`.
std::optional<std::size_t> ordinal_in(const std::string &key, const std::string &stem);

// `key` addressing the document once the instances of `gone` are out of it: an ordinal it names in
// a collection something was taken out of steps down by however many of those stood in front of it.
std::string renumbered(const std::string &key, std::span<const taken> gone);

// What is wrong with taking out `gone`, or nothing: a collection addressed through any instance of
// a collection that loses an instance stands at an ordinal the removal may move.
std::string moved_collection(std::span<const taken> gone);

// What is wrong with writing the bound edits among `changes` beside taking out `gone`, or nothing: a
// value addressed to an instance that goes has nowhere left to land.
std::string written_into_taken(std::span<const edit> changes, std::span<const taken> gone);

}

#endif
