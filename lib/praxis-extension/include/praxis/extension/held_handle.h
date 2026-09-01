#ifndef HPP_GUARD_PRAXIS_EXTENSION_HELD_HANDLE_H
#define HPP_GUARD_PRAXIS_EXTENSION_HELD_HANDLE_H

#include <memory>
#include <string>
#include <stdexcept>
#include <string_view>

namespace praxis {

namespace detail {

std::string absent_handle(std::string_view holder, std::string_view handle);

}

// A holder keeps its share for as long as it lives and reads through it while constructing or on a
// rendered frame, so absence is not a state it can carry. Both ends are named because the refusal
// is read by whoever wired the composition, not by whoever wrote the holder.
template<typename handle_type>
handle_type &held(const std::shared_ptr<handle_type> &share, std::string_view holder, std::string_view handle)
{
    if(!share)
        throw std::invalid_argument(detail::absent_handle(holder, handle));

    return *share;
}

}

#endif
