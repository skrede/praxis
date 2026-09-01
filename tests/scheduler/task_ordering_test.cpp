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

void advance_clock(scheduler &loop, time_point::duration elapsed)
{
    dictated += elapsed;
    REQUIRE(loop.drain().has_value());
}

}

TEST_CASE("equal stepped deadlines preserve registration order", "[scheduler][task][ordering]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    std::vector<std::uint32_t> order;
    std::vector<task_handle> tasks;
    for(std::uint32_t registration = 1; registration <= 4; ++registration)
        tasks.push_back(sim.every(step_period{10ms}, overrun::drop, [&order, registration](step_delta) { order.push_back(registration); }));

    for(std::uint32_t deadline = 0; deadline < 3; ++deadline)
        advance_clock(loop, 10ms);

    REQUIRE(order == std::vector<std::uint32_t>{1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4});
}

TEST_CASE("equal one-shot deadlines preserve registration order once", "[scheduler][task][ordering]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    std::vector<std::uint32_t> order;
    std::vector<task_handle> tasks;
    for(std::uint32_t registration = 1; registration <= 4; ++registration)
        tasks.push_back(sim.after(step_period{10ms}, [&order, registration] { order.push_back(registration); }));

    advance_clock(loop, 10ms);

    REQUIRE(order == std::vector<std::uint32_t>{1, 2, 3, 4});
    for(const task_handle &task : tasks)
        REQUIRE_FALSE(task.active());
}
