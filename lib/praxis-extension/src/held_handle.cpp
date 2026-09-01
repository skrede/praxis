#include "praxis/extension/held_handle.h"

#include <string>
#include <string_view>

namespace praxis::detail {

std::string absent_handle(std::string_view holder, std::string_view handle)
{
    return "praxis: " + std::string(holder) + " was given no " + std::string(handle) + " to hold";
}

}
