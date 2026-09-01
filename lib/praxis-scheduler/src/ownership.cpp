#include "praxis/scheduler/ownership.h"

#include <string>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace praxis::scheduler {

namespace {

std::string wrong_strand(std::string_view object, std::uint32_t owner)
{
    return "praxis: " + std::string(object) + " is owned by strand " + std::to_string(owner) + " and was reached from somewhere else";
}

}

void require_strand(const strand &owner, std::string_view object)
{
    if(!owner.running_here())
        throw std::logic_error(wrong_strand(object, static_cast<std::uint32_t>(owner.id())));
}

}
