#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <cstdint>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

// A servicing thread reads the clock, so the dictated reading is shared rather than plain.
std::atomic<time_point> dictated{time_point{}};

time_point reading()
{
    return dictated.load(std::memory_order_acquire);
}

clock_source dictating()
{
    dictated.store(time_point{}, std::memory_order_release);

    return clock_source{&reading};
}

void advance_clock(scheduler &loop, time_point::duration by)
{
    dictated.store(dictated.load(std::memory_order_acquire) + by, std::memory_order_release);
    REQUIRE(loop.drain().has_value());
}

constexpr auto period = 1ms;
constexpr time_point::duration one_step{1};
const step_period nominal{seconds{period}};

}

TEST_CASE("a deadline advances from the schedule, so a long run accumulates no drift", "[scheduler][deadline]")
{
    constexpr std::uint64_t frames = 600;

    scheduler loop(inline_workers, dictating());
    std::uint64_t ticks    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&ticks](step_delta) { ++ticks; });
    for(std::uint64_t frame = 0; frame < frames; ++frame)
        advance_clock(loop, 1300us);

    REQUIRE(ticks == 780);
    REQUIRE(task.counters().ran == 780);
    REQUIRE(task.counters().dropped == 0);
}

TEST_CASE("a frame exactly one period long produces exactly one tick", "[scheduler][deadline]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t ticks    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&ticks](step_delta) { ++ticks; });
    advance_clock(loop, period);

    REQUIRE(ticks == 1);
    REQUIRE(task.counters().ran == 1);
}

TEST_CASE("a frame one clock step short of a period holds its tick until the next", "[scheduler][deadline]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t ticks    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&ticks](step_delta) { ++ticks; });
    advance_clock(loop, period - one_step);
    REQUIRE(ticks == 0);

    advance_clock(loop, one_step);
    REQUIRE(ticks == 1);
    REQUIRE(task.counters().ran == 1);
}

TEST_CASE("a frame one clock step past a period credits the remainder to the next deadline", "[scheduler][deadline]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t ticks    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&ticks](step_delta) { ++ticks; });
    advance_clock(loop, period + one_step);
    REQUIRE(ticks == 1);

    advance_clock(loop, period - one_step);
    REQUIRE(ticks == 2);
    REQUIRE(task.counters().ran == 2);
}

// A period that is not a binary fraction of a second is exact only if the owed count is taken in
// clock ticks; taken as a ratio of two floating-point seconds it loses a period at this size.
TEST_CASE("a gap of many thousand periods owes every one of them", "[scheduler][deadline]")
{
    stepped_task task(nominal, overrun::catch_up, [](step_delta) {});
    task.arm(time_point{});
    task.run(time_point{} + 65535ms);

    REQUIRE(task.counters().ran + task.counters().dropped == 65535);
    REQUIRE(task.counters().ran + task.counters().dropped != 65534);
    REQUIRE(task.counters().ran == catch_up_limit);
}

TEST_CASE("a high-rate task off the main strand keeps every tick a long frame covers", "[scheduler][deadline]")
{
    constexpr std::uint64_t covered = 8;

    scheduler loop(default_workers(), dictating());
    std::atomic<std::uint64_t> ticks{0};
    std::promise<void> owed;
    std::future<void> reached = owed.get_future();

    const task_handle high = loop.simulation_strand().every(nominal, overrun::catch_up,
                                                            [&ticks, &owed](step_delta)
                                                            {
                                                                if(ticks.fetch_add(1, std::memory_order_relaxed) + 1 == covered)
                                                                    owed.set_value();
                                                            });

    std::uint64_t counted     = 0;
    bool delivered            = false;
    const task_handle framing = loop.main_strand().every(every_step, overrun::drop,
                                                         [&](step_delta)
                                                         {
                                                             dictated.store(dictated.load(std::memory_order_acquire) + covered * period, std::memory_order_release);
                                                             delivered = reached.wait_for(5s) == std::future_status::ready;
                                                             counted   = ticks.load(std::memory_order_relaxed);
                                                             loop.stop();
                                                         });

    REQUIRE(high.valid());
    REQUIRE(framing.valid());
    loop.run();

    REQUIRE(delivered);
    REQUIRE(counted == covered);
}
