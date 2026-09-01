#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <vector>
#include <cstdint>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};

    return clock_source{&reading};
}

constexpr auto period = 1ms;
const step_period nominal{seconds{period}};

}

TEST_CASE("catch_up decomposes a stall into one call per period", "[scheduler][overrun]")
{
    std::vector<seconds> received;
    stepped_task task(nominal, overrun::catch_up, [&received](step_delta delta) { received.push_back(delta.value); });
    task.arm(time_point{});
    task.run(time_point{} + 3 * period);

    REQUIRE(received == std::vector<seconds>{seconds{period}, seconds{period}, seconds{period}});
    REQUIRE(task.counters().ran == 3);
    REQUIRE(task.counters().dropped == 0);
}

TEST_CASE("drop hands a stall over whole and counts the periods it skipped", "[scheduler][overrun]")
{
    std::vector<seconds> received;
    stepped_task task(nominal, overrun::drop, [&received](step_delta delta) { received.push_back(delta.value); });
    task.arm(time_point{});
    task.run(time_point{} + 3 * period);

    REQUIRE(received == std::vector<seconds>{seconds{3 * period}});
    REQUIRE(task.counters().ran == 1);
    REQUIRE(task.counters().dropped == 2);
}

TEST_CASE("the largest overrun is measured from the deadline the fire was scheduled for", "[scheduler][overrun]")
{
    for(const overrun policy : {overrun::catch_up, overrun::drop})
    {
        stepped_task task(nominal, policy, [](step_delta) {});
        task.arm(time_point{});
        task.run(time_point{} + 3 * period);
        REQUIRE(task.counters().worst_lateness == seconds{2 * period});
    }
}

TEST_CASE("the counters name the policy the task carries", "[scheduler][overrun]")
{
    scheduler loop(inline_workers, dictating());
    const task_handle catching = loop.simulation_strand().every(nominal, overrun::catch_up, [](step_delta) {});
    const task_handle dropping = loop.simulation_strand().every(nominal, overrun::drop, [](step_delta) {});
    dictated += 3 * period;
    REQUIRE(loop.drain().has_value());

    REQUIRE(catching.counters().policy == overrun::catch_up);
    REQUIRE(dropping.counters().policy == overrun::drop);
    REQUIRE(catching.counters().ran == 3);
    REQUIRE(dropping.counters().ran == 1);
    REQUIRE(dropping.counters().dropped == 2);
}

TEST_CASE("an absolute-time task carries no policy and drops nothing", "[scheduler][overrun]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t samples  = 0;
    const task_handle task = loop.simulation_strand().sample(nominal, [&samples](sample_time) { ++samples; });
    dictated += 7 * period;
    REQUIRE(loop.drain().has_value());

    REQUIRE(samples == 1);
    REQUIRE_FALSE(task.counters().policy.has_value());
    REQUIRE(task.counters().ran == 1);
    REQUIRE(task.counters().dropped == 0);
}

TEST_CASE("a stall past the bound replays the bound and no more", "[scheduler][overrun]")
{
    std::uint64_t calls = 0;
    stepped_task task(nominal, overrun::catch_up, [&calls](step_delta) { ++calls; }, replay_bound{16});
    task.arm(time_point{});
    task.run(time_point{} + 100 * period);

    REQUIRE(calls == 16);
    REQUIRE(task.counters().ran == 16);
}

TEST_CASE("the ticks the bound refused are counted as dropped", "[scheduler][overrun]")
{
    stepped_task task(nominal, overrun::catch_up, [](step_delta) {}, replay_bound{16});
    task.arm(time_point{});
    task.run(time_point{} + 100 * period);

    REQUIRE(task.counters().dropped == 84);
    REQUIRE(task.counters().ran + task.counters().dropped == 100);
}

TEST_CASE("a stall inside the bound is replayed whole and drops nothing", "[scheduler][overrun]")
{
    std::uint64_t calls = 0;
    stepped_task task(nominal, overrun::catch_up, [&calls](step_delta) { ++calls; }, replay_bound{16});
    task.arm(time_point{});
    task.run(time_point{} + 16 * period);

    REQUIRE(calls == 16);
    REQUIRE(task.counters().ran == 16);
    REQUIRE(task.counters().dropped == 0);
}

TEST_CASE("a bounded fire converges on the next deadline rather than falling further behind", "[scheduler][overrun]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t calls    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&calls](step_delta) { ++calls; }, replay_bound{16});
    dictated += 100 * period;
    REQUIRE(loop.drain().has_value());
    REQUIRE(calls == 16);

    std::vector<std::uint64_t> after;
    for(std::uint64_t fire = 0; fire < 4; ++fire)
    {
        calls = 0;
        dictated += period;
        REQUIRE(loop.drain().has_value());
        after.push_back(calls);
    }

    REQUIRE(after == std::vector<std::uint64_t>{1, 1, 1, 1});
    REQUIRE(task.counters().ran == 20);
    REQUIRE(task.counters().dropped == 84);
}

TEST_CASE("a registration that names no bound carries the default one", "[scheduler][overrun]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t calls    = 0;
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [&calls](step_delta) { ++calls; });
    dictated += 2 * catch_up_limit * period;
    REQUIRE(loop.drain().has_value());

    REQUIRE(calls == catch_up_limit);
    REQUIRE(task.counters().ran == catch_up_limit);
    REQUIRE(task.counters().dropped == catch_up_limit);
}

TEST_CASE("a bound of no ticks is refused rather than admitted", "[scheduler][overrun]")
{
    scheduler loop(inline_workers, dictating());
    const task_handle task = loop.simulation_strand().every(nominal, overrun::catch_up, [](step_delta) {}, replay_bound{0});

    REQUIRE_FALSE(task.valid());
    REQUIRE_FALSE(task.active());
}
