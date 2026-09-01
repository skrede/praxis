#ifndef HPP_GUARD_PRAXIS_CONFIG_ERROR_H
#define HPP_GUARD_PRAXIS_CONFIG_ERROR_H

#include <string>
#include <cstdint>

namespace praxis::config {

enum class error_code : std::uint8_t
{
    absent_source,
    unreadable_source,
    empty_source,
    malformed_source,
    mismatched_space,
    rejected_content,
    absent_key,
    mismatched_kind,
    instance_required,
    unlocatable_key,
    unwritable_target,
};

// The message is the text the layer that refused wrote, carried verbatim rather than rephrased, so
// a diagnostic a person reads names the container, the field and the value the refusal was about.
struct error
{
    error_code code;
    std::string message;
};

const char *error_name(error_code code);

}

#endif
