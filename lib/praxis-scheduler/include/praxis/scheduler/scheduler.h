#ifndef HPP_GUARD_PRAXIS_SCHEDULER_SCHEDULER_H
#define HPP_GUARD_PRAXIS_SCHEDULER_SCHEDULER_H

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/clock.h"
#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/overrun.h"
#include "praxis/scheduler/rejection.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include <memory>
#include <cstdint>

namespace praxis::scheduler {

class pool;

// The worker count under which every strand is serviced inline by whoever calls step(), drain() or
// run().
inline constexpr std::uint32_t inline_workers = 0;

// Never zero, so a scheduler built with it services every strand but the main one without anybody
// advancing it.
std::uint32_t default_workers();

// The owner of the clock. Strand choice rotates after each service. Within a strand, ready work
// runs first when contention begins, then ready and due work alternate while both remain eligible.
class scheduler
{
public:
    explicit scheduler(std::uint32_t workers);
    scheduler(std::uint32_t workers, clock_source clock);

    scheduler(const scheduler &)            = delete;
    scheduler(scheduler &&)                 = delete;
    scheduler &operator=(const scheduler &) = delete;
    scheduler &operator=(scheduler &&)      = delete;

    ~scheduler();

    // Main is the one strand the workers never service, because the operating system pins window
    // event pumping to the thread the program started on. Every other strand is what make_strand()
    // returns, including the simulation one, which the constructor makes and keeps.
    strand main_strand() const;
    strand simulation_strand() const;

    expected<strand, rejection> make_strand();

    // The strand finishes what it already holds, admits nothing further from the moment this is
    // taken, and runs the acknowledgment last and on itself; the calling thread waits for none of
    // it.
    expected<void, rejection> retire_strand(strand target, detail::move_only_function<void()> acknowledgment);

    // True when this call ran a unit of work. A drain reports on the whole pool -- no handler
    // running, none queued and no deadline passed -- which a task registered with every_step never
    // reaches, and refuses a call made from inside a handler it is running.
    bool step();
    expected<void, rejection> drain();

    void run();
    void stop();

private:
    friend class strand;

    // A strand is a capability rather than a view of the scheduler, so asking which strand is the
    // main one changes nothing and is const while the handle it hands out is not. The address is
    // recorded once here rather than cast back at every hand-out.
    scheduler *m_self;
    strand m_simulation;
    std::shared_ptr<pool> m_pool;

    bool running_here(strand_id on) const;

    expected<void, rejection> post(strand_id on, detail::move_only_function<void()> work);

    task_handle after(strand_id on, step_period delay, detail::move_only_function<void()> run);
    task_handle every(strand_id on, step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound);
    task_handle sample(strand_id on, step_period period, detail::move_only_function<void(sample_time)> work);
};

}

#endif
