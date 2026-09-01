#ifndef HPP_GUARD_PRAXIS_SCHEDULER_OWNERSHIP_H
#define HPP_GUARD_PRAXIS_SCHEDULER_OWNERSHIP_H

#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/rejection.h"

#include "praxis/compat/expected.h"

#include <utility>
#include <string_view>
#include <type_traits>

namespace praxis::scheduler {

// The module's two channels divide here as they do everywhere else in it: a state the caller is
// expected to handle is returned, and a programming error throws. Offering work to a strand that
// admits none is a state; reading an object from a strand that does not own it is not. The refusal
// is unconditional, so it is as true of a released binary as of a test one.
void require_strand(const strand &owner, std::string_view object);

// State one strand serializes. No member names the value, so no expression outside a handler on
// that strand reads or writes it, and the gate hands it to the callable as a parameter rather than
// leaving the callable to capture something whose lifetime is shorter than the post.
template<typename value_type>
class strand_owned
{
public:
    template<typename... arguments>
    explicit strand_owned(strand owner, arguments &&...initial)
            : m_owner(owner)
            , m_value(std::forward<arguments>(initial)...)
    {
    }

    strand_owned(const strand_owned &)            = delete;
    strand_owned(strand_owned &&)                 = delete;
    strand_owned &operator=(const strand_owned &) = delete;
    strand_owned &operator=(strand_owned &&)      = delete;

    ~strand_owned() = default;

    // Always posted and never run on the calling thread, so a handler cannot re-enter a value
    // another handler on the same strand is already inside.
    template<typename operation>
    expected<void, rejection> with(operation work)
    {
        static_assert(!std::is_reference_v<std::invoke_result_t<operation &, value_type &>>, "praxis: a strand-owned value must not leave its gate by reference");

        return m_owner.post([this, work = std::move(work)]() mutable { work(m_value); });
    }

private:
    strand m_owner;
    value_type m_value;
};

}

#endif
