#include "engine.h"

#include "praxis/config/error.h"

namespace praxis::config {

const char *error_name(error_code code)
{
    switch(code)
    {
        case error_code::absent_source:
            return "absent_source";
        case error_code::unreadable_source:
            return "unreadable_source";
        case error_code::empty_source:
            return "empty_source";
        case error_code::malformed_source:
            return "malformed_source";
        case error_code::mismatched_space:
            return "mismatched_space";
        case error_code::rejected_content:
            return "rejected_content";
        case error_code::absent_key:
            return "absent_key";
        case error_code::mismatched_kind:
            return "mismatched_kind";
        case error_code::instance_required:
            return "instance_required";
        case error_code::unlocatable_key:
            return "unlocatable_key";
        case error_code::unwritable_target:
            return "unwritable_target";
    }
    return "unknown";
}

// Every refusal the engine reports that this module has a name for keeps the engine's own message;
// the rest arrive as a malformed document, which is what a refused fold of one file amounts to.
error translated(const nucleus::error &reason)
{
    switch(reason.code)
    {
        case nucleus::errc::unreadable_source:
            return error{error_code::unreadable_source, reason.message};
        case nucleus::errc::absent_key:
            return error{error_code::absent_key, reason.message};
        case nucleus::errc::index_required:
            return error{error_code::instance_required, reason.message};
        case nucleus::errc::missing_converter:
        case nucleus::errc::mismatched_type:
            return error{error_code::mismatched_kind, reason.message};
        case nucleus::errc::schema_violation:
        case nucleus::errc::failed_conversion:
        case nucleus::errc::invalid_selection:
            return error{error_code::rejected_content, reason.message};
        default:
            return error{error_code::malformed_source, reason.message};
    }
}

}
