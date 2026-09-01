#ifndef HPP_GUARD_PRAXIS_SCHEDULER_POOL_H
#define HPP_GUARD_PRAXIS_SCHEDULER_POOL_H

#include "queue.h"

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/clock.h"
#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/rejection.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include <mutex>
#include <thread>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <condition_variable>

namespace praxis::scheduler {

std::size_t seat(strand_id on);

// The half-open range of strand indices one caller services.
struct served
{
    std::size_t first;
    std::size_t limit;
};

// Whether withdrawing a task holds the caller until an invocation already inside the handler has
// returned.
enum class withdrawal : std::uint8_t
{
    marked,
    joined
};

// One invocation in flight. A cancellation that arrives while it runs marks it rather than
// forgetting it, so the strand can refuse the rearm and a waiter can tell when it has returned. The
// tallies are copied out as the task leaves its queue, because the handler mutates its own while
// the mutex a reader holds is released.
struct running_entry
{
    bool cancelled;
    std::uint64_t id;
    task_counters tally;
};

// The strand table and the workers over it. One mutex guards every queue, and the whole of the
// protocol around it lives here: an admission notifies after it has pushed, an advance lets the
// mutex go before it invokes a handler and takes it back afterwards, a wait re-reads the queues
// under it so nothing that arrived meanwhile is slept through, and a count of what is queued or
// running is what makes quiescence a question about the whole table rather than about one caller's
// own strands.
class pool
{
    class worker_start_guard
    {
    public:
        explicit worker_start_guard(pool &owner);
        ~worker_start_guard();

        void complete();

    private:
        bool m_active;
        pool &m_owner;
    };

    class service_guard
    {
    public:
        service_guard(pool &owner, std::unique_lock<std::mutex> &held, strand_id on, std::uint64_t running);
        ~service_guard();

        void complete();

    private:
        bool m_active;
        pool &m_owner;
        strand_id m_on;
        std::uint64_t m_running;
        std::unique_lock<std::mutex> &m_held;
    };

public:
    pool(std::uint32_t workers, clock_source clock);

    pool(const pool &)            = delete;
    pool(pool &&)                 = delete;
    pool &operator=(const pool &) = delete;
    pool &operator=(pool &&)      = delete;

    ~pool();

    strand_id main() const;
    bool known(strand_id on) const;

    expected<strand_id, rejection> make();
    expected<void, rejection> retire(strand_id on, detail::move_only_function<void()> acknowledgment);

    expected<void, rejection> post(strand_id on, detail::move_only_function<void()> work);

    std::uint64_t arm(strand_id on, scheduled_task work);
    void cancel(std::uint64_t id);
    expected<void, rejection> join(std::uint64_t id);
    bool active(std::uint64_t id) const;
    task_counters counters(std::uint64_t id) const;

    bool step();
    expected<void, rejection> drain();

    void run();
    void stop();

private:
    bool m_stopped;
    bool m_threaded;
    bool m_signalled;
    clock_source m_clock;
    std::uint32_t m_waiting;
    std::uint64_t m_sequence;
    mutable std::mutex m_mutex;
    std::uint64_t m_generation;
    std::size_t m_inline_cursor;
    std::size_t m_worker_cursor;
    std::uint64_t m_outstanding;
    std::condition_variable m_wakeup;
    std::condition_variable m_completion;
    std::vector<std::thread> m_workers;
    std::vector<strand_queue> m_strands;
    std::vector<running_entry> m_running;

    bool stopped() const;
    bool in_flight(std::uint64_t id) const;

    void attend(bool worker);
    bool advance(bool worker);
    bool advance_one(std::unique_lock<std::mutex> &held, strand_id on, time_point now, std::size_t &cursor, std::size_t next);
    void withdraw(std::uint64_t id, withdrawal mode);

    // These functions are called with m_mutex held. Servicing releases it around the handler, so
    // none may keep a reference into the strand table across that gap.
    bool run_ready(std::unique_lock<std::mutex> &held, strand_id on);
    bool run_due(std::unique_lock<std::mutex> &held, strand_id on, time_point now);
    void abandon(std::unique_lock<std::mutex> &held, strand_id on, std::uint64_t running);
    void finish_ready(std::unique_lock<std::mutex> &held, strand_id on);
    void finish_due(std::unique_lock<std::mutex> &held, strand_id on, armed_task &entry);
    bool mark_cancelled(std::uint64_t id);
    void await_completion(std::unique_lock<std::mutex> &held, std::uint64_t id);
    void park(std::unique_lock<std::mutex> &held, bool worker);
    void admit(strand_id on, detail::move_only_function<void()> &&work);
    void join_workers();
    discarded_work evacuate(std::size_t index);
    served span_held(bool worker) const;
    bool pending(served range) const;
    bool quiet() const;
    bool publish();
};

}

#endif
