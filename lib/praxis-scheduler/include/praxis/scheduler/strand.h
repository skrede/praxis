#ifndef HPP_GUARD_PRAXIS_SCHEDULER_STRAND_H
#define HPP_GUARD_PRAXIS_SCHEDULER_STRAND_H

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/clock.h"
#include "praxis/scheduler/overrun.h"
#include "praxis/scheduler/rejection.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include <cstdint>

namespace praxis::scheduler {

class scheduler;

// A strong integral identity said in the language: a scoped enumeration with no enumerators
// converts to nothing on its own and needs no wrapper to say so.
enum class strand_id : std::uint32_t
{
};

// A serialization guarantee multiplexed over the workers, not a thread. The handle is a cheap
// copyable value because a holder keeps one as a member and hands one on by value; the owning
// pointer is the modelled absence a default-constructed handle carries and reports through valid(),
// never an ownership claim, since whoever built the scheduler outlives every handle to it.
class strand
{
public:
    strand();

    bool valid() const;
    bool running_here() const;

    strand_id id() const;

    expected<void, rejection> post(detail::move_only_function<void()> work) const;

    task_handle after(step_period delay, detail::move_only_function<void()> run) const;
    task_handle every(step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound = replay_bound{catch_up_limit}) const;
    task_handle sample(step_period period, detail::move_only_function<void(sample_time)> work) const;

private:
    friend class scheduler;

    strand_id m_id;
    scheduler *m_owner;

    strand(scheduler &owner, strand_id id);
};

}

#endif
