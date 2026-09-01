#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <vector>
#include <cstdint>
#include <utility>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

std::atomic<std::int64_t> clock_ms{0};
std::atomic<bool> advancing{false};

time_point reading()
{
    const std::int64_t value = advancing ? clock_ms.fetch_add(1) : clock_ms.load();
    return time_point{} + std::chrono::milliseconds{value};
}

clock_source dictating(std::int64_t initial = 0)
{
    clock_ms  = initial;
    advancing = false;
    return clock_source{&reading};
}

void post_blocked(strand target, std::promise<void> &entered, std::shared_future<void> released, std::atomic<std::uint32_t> &active)
{
    REQUIRE(target.post(
                          [&entered, &active, released]
                          {
                              active.fetch_add(1);
                              entered.set_value();
                              released.wait_for(2s);
                              active.fetch_sub(1);
                          })
                    .has_value());
}

void post_observer(strand target, std::promise<void> &done, std::atomic<std::uint32_t> &active, std::atomic<bool> &overlap)
{
    REQUIRE(target.post(
                          [&done, &active, &overlap]
                          {
                              overlap = active.fetch_add(1) != 0;
                              active.fetch_sub(1);
                              done.set_value();
                          })
                    .has_value());
}

auto blocking_task(std::atomic<std::uint32_t> &calls, std::promise<void> &entered, std::shared_future<void> released, std::promise<void> &repeated)
{
    return [&calls, &entered, released, &repeated](step_delta)
    {
        if(++calls == 1)
        {
            entered.set_value();
            released.wait_for(2s);
        }
        else
            repeated.set_value();
    };
}

}

TEST_CASE("one strand never overlaps while another strand progresses", "[scheduler][pool]")
{
    scheduler loop(2);
    const strand first = loop.simulation_strand();
    const strand other = *loop.make_strand();
    std::promise<void> entered, release, same, elsewhere;
    std::shared_future<void> released = release.get_future().share();
    std::future<void> same_done       = same.get_future();
    std::atomic<std::uint32_t> active{0};
    std::atomic<bool> overlap{false};

    post_blocked(first, entered, released, active);
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    post_observer(first, same, active, overlap);
    REQUIRE(other.post([&] { elsewhere.set_value(); }).has_value());

    REQUIRE(elsewhere.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(same_done.wait_for(20ms) == std::future_status::timeout);
    release.set_value();
    REQUIRE(same_done.wait_for(2s) == std::future_status::ready);
    REQUIRE_FALSE(overlap);
}

TEST_CASE("retirement acknowledgment stays behind a running handler", "[scheduler][pool]")
{
    scheduler loop(2);
    const strand sim = loop.simulation_strand();
    std::promise<void> entered, release, acknowledged;
    std::shared_future<void> released = release.get_future().share();
    std::atomic<std::uint32_t> active{0};
    post_blocked(sim, entered, released, active);
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(loop.retire_strand(sim, [&] { acknowledged.set_value(); }).has_value());
    std::future<void> done = acknowledged.get_future();
    REQUIRE(done.wait_for(20ms) == std::future_status::timeout);
    release.set_value();
    REQUIRE(done.wait_for(2s) == std::future_status::ready);
}

TEST_CASE("drain wakes when running work publishes a new generation", "[scheduler][pool]")
{
    scheduler loop(1);
    const strand sim = loop.simulation_strand();
    std::promise<void> entered, release;
    std::shared_future<void> released = release.get_future().share();
    std::atomic<std::uint32_t> active{0};
    post_blocked(sim, entered, released, active);
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    std::future<praxis::expected<void, rejection>> drained = std::async(std::launch::async, [&] { return loop.drain(); });
    REQUIRE(drained.wait_for(20ms) == std::future_status::timeout);
    release.set_value();
    REQUIRE(drained.wait_for(2s) == std::future_status::ready);
    REQUIRE(drained.get().has_value());
}

TEST_CASE("task captures are destroyed outside the pool mutex", "[scheduler][pool]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    std::atomic<std::uint32_t> fires{0};
    task_handle canceled = sim.after(step_period{1s}, [&] { ++fires; });
    task_handle holder   = sim.after(step_period{1s}, [captured = std::move(canceled)] {});
    holder.cancel();

    task_handle completed = sim.after(step_period{20ms}, [&] { ++fires; });
    task_handle one_shot  = sim.after(step_period{10ms}, [captured = std::move(completed)] {});
    clock_ms              = 10;
    REQUIRE(loop.drain().has_value());
    clock_ms = 1000;
    REQUIRE(loop.drain().has_value());
    REQUIRE(fires == 0);
}

TEST_CASE("stop prevents queued and due work from beginning", "[scheduler][pool]")
{
    scheduler loop(inline_workers, dictating());
    std::uint32_t ran = 0;
    REQUIRE(loop.main_strand().post([&] { ++ran; }).has_value());
    const task_handle due = loop.simulation_strand().after(every_step, [&] { ++ran; });
    loop.stop();
    REQUIRE_FALSE(loop.step());
    const praxis::expected<void, rejection> drained = loop.drain();
    REQUIRE_FALSE(drained.has_value());
    REQUIRE(drained.error() == rejection::scheduler_stopped);
    REQUIRE(ran == 0);
}

TEST_CASE("sampled peers receive one shared service time", "[scheduler][pool]")
{
    scheduler loop(inline_workers, dictating());
    std::vector<seconds> samples;
    std::vector<task_handle> tasks;
    for(std::uint32_t index = 0; index < 2; ++index)
        tasks.push_back(loop.simulation_strand().sample(step_period{10ms}, [&](sample_time time) { samples.push_back(time.value); }));
    clock_ms  = 10;
    advancing = true;
    REQUIRE(loop.step());
    REQUIRE(loop.step());
    REQUIRE(samples == std::vector<seconds>{seconds{11ms}, seconds{11ms}});
}

TEST_CASE("cancellation while running prevents rearm", "[scheduler][pool]")
{
    scheduler loop(1);
    const strand sim = loop.simulation_strand();
    std::promise<void> entered, release, acknowledged, repeated;
    std::shared_future<void> released = release.get_future().share();
    std::atomic<std::uint32_t> calls{0};
    task_handle task = sim.every(every_step, overrun::drop, blocking_task(calls, entered, released, repeated));
    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    task.cancel();
    release.set_value();
    REQUIRE(sim.post([&] { acknowledged.set_value(); }).has_value());
    REQUIRE(acknowledged.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(repeated.get_future().wait_for(20ms) == std::future_status::timeout);
    REQUIRE(calls == 1);
}
