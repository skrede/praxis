#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <string>
#include <cstdint>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

std::atomic<std::int64_t> dictated_ms{0};

time_point reading()
{
    return time_point{} + std::chrono::milliseconds{dictated_ms.load()};
}

clock_source dictating()
{
    dictated_ms = 0;
    return clock_source{&reading};
}

struct identity_observation
{
    bool own_here;
    bool nested_ran;
    bool foreign_here;
    bool foreign_drained;
    bool nested_admitted;
    bool own_rejected;
};

void inspect_identity(scheduler &first, scheduler &second, strand first_main, strand second_main, identity_observation &seen)
{
    seen.own_here                               = first_main.running_here();
    seen.foreign_here                           = second_main.running_here();
    seen.nested_admitted                        = second_main.post([&] { seen.nested_ran = second_main.running_here(); }).has_value();
    seen.foreign_drained                        = second.drain().has_value();
    const praxis::expected<void, rejection> own = first.drain();
    seen.own_rejected                           = !own.has_value() && own.error() == rejection::reentrant_drain;
}

void saturate(strand target, std::uint32_t &calls, bool &admitted)
{
    const praxis::expected<void, rejection> posted = target.post(
            [target, &calls, &admitted]
            {
                ++calls;
                saturate(target, calls, admitted);
            });
    admitted = admitted && posted.has_value();
}

}

TEST_CASE("running identity includes the scheduler", "[scheduler][integrity]")
{
    scheduler first(inline_workers, dictating());
    scheduler second(inline_workers, dictating());
    identity_observation seen{false, false, false, false, false, false};
    const strand first_main  = first.main_strand();
    const strand second_main = second.main_strand();
    REQUIRE(first_main.post([&] { inspect_identity(first, second, first_main, second_main, seen); }).has_value());
    REQUIRE(first.step());
    REQUIRE(seen.own_here);
    REQUIRE_FALSE(seen.foreign_here);
    REQUIRE(seen.nested_admitted);
    REQUIRE(seen.foreign_drained);
    REQUIRE(seen.nested_ran);
    REQUIRE(seen.own_rejected);
}

TEST_CASE("saturated strands rotate at each service", "[scheduler][integrity]")
{
    scheduler loop(inline_workers, dictating());
    std::uint32_t first = 0, second = 0;
    bool admitted = true;
    saturate(loop.simulation_strand(), first, admitted);
    saturate(*loop.make_strand(), second, admitted);
    for(std::uint32_t service = 0; service < 12; ++service)
        REQUIRE(loop.step());
    loop.stop();
    REQUIRE(admitted);
    REQUIRE(first == 6);
    REQUIRE(second == 6);
}

TEST_CASE("ready posts and due tasks receive bounded service", "[scheduler][integrity]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim    = loop.simulation_strand();
    std::uint32_t ready = 0, due = 0;
    bool admitted = true;
    saturate(sim, ready, admitted);
    const task_handle task = sim.every(every_step, overrun::drop, [&](step_delta) { ++due; });
    for(std::uint32_t service = 0; service < 25; ++service)
        REQUIRE(loop.step());
    loop.stop();
    REQUIRE(admitted);
    REQUIRE(ready == 13);
    REQUIRE(due == 12);
}

TEST_CASE("contention epochs preserve ready-first alternation", "[scheduler][integrity]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    std::string order;
    const task_handle first = sim.after(step_period{10ms}, [&] { order += 'D'; });
    task_handle second      = sim.after(step_period{10ms}, [&] { order += 'D'; });
    REQUIRE(sim.post([&] { order += 'U'; }).has_value());
    REQUIRE(loop.step());
    dictated_ms = 10;
    REQUIRE(sim.post([&] { order += 'R'; }).has_value());
    REQUIRE(loop.step());
    REQUIRE(sim.post([&] { order += 'R'; }).has_value());
    REQUIRE(loop.step());
    REQUIRE(loop.step());
    second.cancel();
    const task_handle third = sim.after(every_step, [&] { order += 'D'; });
    REQUIRE(sim.post([&] { order += 'R'; }).has_value());
    REQUIRE(loop.step());
    REQUIRE(loop.step());
    REQUIRE(order == "URDRRD");
}

TEST_CASE("stop evacuates a callback capturing its own task", "[scheduler][integrity]")
{
    std::future<bool> destroyed = std::async(std::launch::async,
                                             []
                                             {
                                                 scheduler loop(inline_workers, dictating());
                                                 task_handle task = loop.simulation_strand().after(step_period{1s}, [] {});
                                                 return loop.main_strand().post([captured = std::move(task)] {}).has_value();
                                             });
    REQUIRE((destroyed.wait_for(2s) == std::future_status::ready && destroyed.get()));
}

TEST_CASE("refused task destroys its callback outside the pool mutex", "[scheduler][integrity]")
{
    std::future<bool> refused = std::async(std::launch::async,
                                           []
                                           {
                                               scheduler loop(inline_workers, dictating());
                                               task_handle task = loop.simulation_strand().after(step_period{1s}, [] {});
                                               const step_period invalid{seconds{std::numeric_limits<double>::infinity()}};
                                               return !loop.simulation_strand().after(invalid, [captured = std::move(task)] {}).valid();
                                           });
    REQUIRE((refused.wait_for(2s) == std::future_status::ready && refused.get()));
}

TEST_CASE("evacuation may destroy a foreign task receipt", "[scheduler][integrity]")
{
    scheduler owner(inline_workers, dictating());
    std::uint32_t fires         = 0;
    std::future<bool> destroyed = std::async(std::launch::async,
                                             [&]
                                             {
                                                 scheduler doomed(inline_workers);
                                                 task_handle task = owner.simulation_strand().after(step_period{1s}, [&] { ++fires; });
                                                 return doomed.main_strand().post([captured = std::move(task)] {}).has_value();
                                             });
    REQUIRE((destroyed.wait_for(2s) == std::future_status::ready && destroyed.get()));
    dictated_ms = 1000;
    REQUIRE(owner.drain().has_value());
    REQUIRE(fires == 0);
}

TEST_CASE("stop races a running handler without starting its follower", "[scheduler][integrity]")
{
    scheduler loop(1);
    const strand sim = loop.simulation_strand();
    std::promise<void> entered, release, finished;
    std::shared_future<void> released = release.get_future().share();
    std::atomic<std::uint32_t> follower{0};
    const task_handle running = sim.every(every_step, overrun::drop,
                                          [&](step_delta)
                                          {
                                              entered.set_value();
                                              released.wait_for(2s);
                                              finished.set_value();
                                          });
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(sim.post([&] { ++follower; }).has_value());
    std::future<void> stopped = std::async(std::launch::async, [&] { loop.stop(); });
    REQUIRE(stopped.wait_for(2s) == std::future_status::ready);
    REQUIRE(running.valid());
    REQUIRE(running.active());
    release.set_value();
    REQUIRE(finished.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(follower.load() == 0);
}
