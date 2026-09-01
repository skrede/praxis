#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
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

// A period of exactly 2^-9 s: a whole number of clock ticks and a binary fraction of a second at
// once, so counting periods over an interval is exact on both sides of the conversion.
constexpr time_point::duration quantum{1'953'125};
const seconds period{quantum};

std::uint64_t periods_of(std::uint64_t count)
{
    std::uint64_t calls = 0;
    stepped_task task(step_period{period}, overrun::catch_up, [&calls](step_delta delta) { calls += delta.value == period; });
    task.arm(time_point{});
    task.run(time_point{} + quantum * count);

    return calls;
}

}

TEST_CASE("a catch_up replay delivers every period it owes below the cap", "[scheduler][replay]")
{
    REQUIRE(periods_of(1) == 1);
    REQUIRE(periods_of(37) == 37);
    REQUIRE(periods_of(catch_up_limit - 1) == catch_up_limit - 1);
    REQUIRE(periods_of(catch_up_limit) == catch_up_limit);
}

TEST_CASE("a catch_up replay is capped and forgives the remainder", "[scheduler][replay]")
{
    REQUIRE(periods_of(catch_up_limit + 1) == catch_up_limit);
    REQUIRE(periods_of(catch_up_limit * 16) == catch_up_limit);
    REQUIRE(periods_of(3'000'000) == catch_up_limit);
}

TEST_CASE("a service gap no rate of task can turn unbounded", "[scheduler][replay]")
{
    std::uint64_t calls = 0;
    stepped_task task(step_period{seconds{1e-6}}, overrun::catch_up, [&calls](step_delta) { ++calls; });
    task.arm(time_point{});
    task.run(time_point{} + 30s);
    REQUIRE(calls == catch_up_limit);
}

TEST_CASE("a capped replay leaves no backlog behind it", "[scheduler][replay]")
{
    std::uint64_t calls        = 0;
    const time_point overshoot = time_point{} + quantum * (catch_up_limit + 5000);
    stepped_task task(step_period{period}, overrun::catch_up, [&calls](step_delta) { ++calls; });
    task.arm(time_point{});
    task.run(overshoot);
    REQUIRE(calls == catch_up_limit);
    REQUIRE(task.due() == overshoot + quantum);

    calls = 0;
    task.run(overshoot + quantum);
    REQUIRE(calls == 1);
}

TEST_CASE("drop hands the whole interval over at once whatever its size", "[scheduler][replay]")
{
    seconds received{};
    std::uint64_t calls = 0;
    stepped_task task(step_period{period}, overrun::drop,
                      [&](step_delta delta)
                      {
                          ++calls;
                          received = delta.value;
                      });
    task.arm(time_point{});
    task.run(time_point{} + 30s);
    REQUIRE(calls == 1);
    REQUIRE(received == seconds{30s});
}

TEST_CASE("the scheduler caps a replay a long service gap owes", "[scheduler][replay]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t calls = 0;
    seconds widest{};
    const task_handle task = loop.simulation_strand().every(step_period{period}, overrun::catch_up,
                                                            [&](step_delta delta)
                                                            {
                                                                ++calls;
                                                                widest = delta.value > widest ? delta.value : widest;
                                                            });
    dictated += quantum * (catch_up_limit * 4);
    REQUIRE(loop.drain().has_value());
    REQUIRE(calls == catch_up_limit);
    REQUIRE(widest == period);

    calls = 0;
    dictated += quantum;
    REQUIRE(loop.drain().has_value());
    REQUIRE(calls == 1);
}

TEST_CASE("the scheduler replays a short service gap whole", "[scheduler][replay]")
{
    scheduler loop(inline_workers, dictating());
    std::uint64_t calls    = 0;
    const task_handle task = loop.simulation_strand().every(step_period{period}, overrun::catch_up, [&calls](step_delta) { ++calls; });
    dictated += quantum * 17;
    REQUIRE(loop.drain().has_value());
    REQUIRE(calls == 17);

    calls = 0;
    dictated += quantum * 4;
    REQUIRE(loop.drain().has_value());
    REQUIRE(calls == 4);
}
