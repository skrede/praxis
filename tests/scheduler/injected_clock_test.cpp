#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <cstdint>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

std::atomic<std::int64_t> dictated_ms{0};
std::atomic<std::uint64_t> readings{0};

// A fabricated reading in a domain of its own: nothing relates it to the steady clock a timed wait
// is measured against.
time_point reading()
{
    readings.fetch_add(1, std::memory_order_relaxed);

    return time_point{} + std::chrono::milliseconds{dictated_ms.load()};
}

clock_source dictating(std::int64_t initial)
{
    dictated_ms = initial;
    readings    = 0;

    return clock_source{&reading};
}

}

TEST_CASE("a worker with nothing due does not spin on an injected clock", "[scheduler][clock]")
{
    scheduler loop(1, dictating(0));
    const task_handle distant = loop.simulation_strand().after(step_period{seconds{3600.0}}, [] {});
    REQUIRE(distant.valid());

    std::this_thread::sleep_for(200ms);
    const std::uint64_t observed = readings.load();
    loop.stop();

    CAPTURE(observed);
    REQUIRE(observed < 1000);
}

TEST_CASE("a worker parked against an injected clock still wakes for work that arrives", "[scheduler][clock]")
{
    scheduler loop(1, dictating(0));
    const task_handle distant = loop.simulation_strand().after(step_period{seconds{3600.0}}, [] {});
    std::promise<void> ran;
    std::this_thread::sleep_for(20ms);

    REQUIRE(loop.simulation_strand().post([&] { ran.set_value(); }).has_value());
    REQUIRE(ran.get_future().wait_for(2s) == std::future_status::ready);
    loop.stop();
}

TEST_CASE("a worker holds a task until the injected clock reaches its deadline", "[scheduler][clock]")
{
    scheduler loop(1, dictating(0));
    std::promise<void> fired;
    const task_handle task = loop.simulation_strand().after(step_period{seconds{0.050}}, [&] { fired.set_value(); });
    std::future<void> done = fired.get_future();

    REQUIRE(done.wait_for(100ms) == std::future_status::timeout);
    dictated_ms = 50;
    REQUIRE(loop.simulation_strand().post([] {}).has_value());
    REQUIRE(done.wait_for(2s) == std::future_status::ready);
    loop.stop();
}
