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

}
