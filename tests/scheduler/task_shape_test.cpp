#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <vector>
#include <cstdint>
#include <type_traits>

using namespace praxis::scheduler;
using namespace std::chrono_literals;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating(time_point initial = time_point{})
{
    dictated = initial;
    return clock_source{&reading};
}

void advance_clock(scheduler &loop, time_point::duration elapsed)
{
    dictated += elapsed;
    REQUIRE(loop.drain().has_value());
}

seconds one_tick()
{
    return seconds{time_point::duration{1}};
}

seconds largest_safe_period()
{
    return seconds{std::nextafter(seconds{time_point::duration::max()}.count(), 0.0)};
}

void require_refused(task_handle task)
{
    REQUIRE_FALSE((task.valid() || task.active()));
}

}

TEST_CASE("the three task shapes are distinct move-only types", "[scheduler][task]")
{
    STATIC_REQUIRE_FALSE(std::is_same_v<sampled_task, stepped_task>);
    STATIC_REQUIRE_FALSE(std::is_same_v<sampled_task, one_shot_task>);
    STATIC_REQUIRE_FALSE(std::is_same_v<stepped_task, one_shot_task>);
    STATIC_REQUIRE(std::is_move_constructible_v<sampled_task>);
    STATIC_REQUIRE(std::is_move_constructible_v<stepped_task>);
    STATIC_REQUIRE(std::is_move_constructible_v<one_shot_task>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<sampled_task>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<stepped_task>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<one_shot_task>);
    STATIC_REQUIRE(std::is_move_constructible_v<task_handle>);
    STATIC_REQUIRE(std::is_move_assignable_v<task_handle>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<task_handle>);
    STATIC_REQUIRE_FALSE(std::is_copy_assignable_v<task_handle>);
}

TEST_CASE("invalid periods and delays are refused", "[scheduler][task]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim      = loop.simulation_strand();
    const seconds tick    = one_tick();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::array invalid{seconds{-1.0}, seconds{std::nan("")}, seconds{infinity}, seconds{-infinity}, tick / 2, seconds{std::numeric_limits<double>::max()}};
    for(const seconds value : invalid)
    {
        CAPTURE(value.count());
        require_refused(sim.sample(step_period{value}, [](sample_time) {}));
        require_refused(sim.every(step_period{value}, overrun::drop, [](step_delta) {}));
        require_refused(sim.after(step_period{value}, [] {}));
    }
}

TEST_CASE("zero and one-tick boundaries preserve their shape contracts", "[scheduler][task]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    require_refused(sim.sample(every_step, [](sample_time) {}));
    std::uint32_t steps = 0;
    task_handle stepped = sim.every(every_step, overrun::drop, [&steps](step_delta delta) { steps += delta.value == seconds::zero(); });
    REQUIRE(loop.step());
    REQUIRE(steps == 1);
    REQUIRE(stepped.active());
    stepped.cancel();
    std::uint32_t shots    = 0;
    const task_handle shot = sim.after(every_step, [&shots] { ++shots; });
    REQUIRE(shot.valid());
    REQUIRE(loop.drain().has_value());
    REQUIRE(shots == 1);
    REQUIRE_FALSE(shot.active());
}

TEST_CASE("one clock tick is accepted by every task shape", "[scheduler][task]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();
    const step_period period{one_tick()};
    std::uint32_t fires = 0;
    std::vector<task_handle> tasks;
    tasks.push_back(sim.sample(period, [&fires](sample_time) { ++fires; }));
    tasks.push_back(sim.every(period, overrun::drop, [&fires](step_delta) { ++fires; }));
    tasks.push_back(sim.after(period, [&fires] { ++fires; }));
    for(const task_handle &task : tasks)
        REQUIRE(task.valid());
    advance_clock(loop, time_point::duration{1});
    REQUIRE(fires == 3);
}

TEST_CASE("representable deadline boundaries fire and saturate safely", "[scheduler][task]")
{
    const seconds largest = largest_safe_period();
    std::uint32_t samples = 0;
    sampled_task sample(step_period{largest}, [&samples](sample_time) { ++samples; });
    sample.arm(time_point{});
    REQUIRE(sample.due() != time_point::max());
    sample.run(sample.due());
    REQUIRE(samples == 1);
    REQUIRE(sample.due() == time_point::max());
    std::uint32_t steps = 0;
    stepped_task step(step_period{largest}, overrun::catch_up, [&steps](step_delta) { ++steps; });
    step.arm(time_point{});
    REQUIRE(step.due() != time_point::max());
    step.run(step.due());
    REQUIRE(steps == 1);
    REQUIRE(step.due() == time_point::max());
    std::uint32_t shots = 0;
    one_shot_task shot(step_period{largest}, [&shots] { ++shots; });
    shot.arm(time_point{});
    REQUIRE(shot.due() != time_point::max());
    shot.run(shot.due());
    REQUIRE(shots == 1);
}

TEST_CASE("registration near the clock maximum refuses saturated deadlines", "[scheduler][task]")
{
    const time_point::duration tick{1};
    scheduler loop(inline_workers, dictating(time_point::max() - tick * 2));
    const strand sim           = loop.simulation_strand();
    const task_handle terminal = sim.every(step_period{seconds{tick}}, overrun::drop, [](step_delta) {});
    REQUIRE(terminal.active());
    advance_clock(loop, tick);
    REQUIRE_FALSE(terminal.active());
    const step_period overflowing{seconds{tick * 2}};
    require_refused(sim.sample(overflowing, [](sample_time) {}));
    require_refused(sim.every(overflowing, overrun::drop, [](step_delta) {}));
    require_refused(sim.after(overflowing, [] {}));
}

TEST_CASE("the first stepped delta is measured from registration", "[scheduler][task]")
{
    scheduler loop(inline_workers, dictating());
    seconds received{};
    const task_handle task = loop.simulation_strand().every(step_period{seconds{0.010}}, overrun::drop, [&received](step_delta delta) { received = delta.value; });
    advance_clock(loop, std::chrono::milliseconds{37});
    REQUIRE(task.valid());
    REQUIRE(received == seconds{std::chrono::milliseconds{37}});
    REQUIRE(received != seconds{std::chrono::milliseconds{27}});
    REQUIRE(received != seconds{std::chrono::milliseconds{10}});
}

TEST_CASE("a late sampled task fires once at current time and skips missed deadlines", "[scheduler][task]")
{
    std::vector<seconds> received;
    sampled_task task(step_period{10ms}, [&received](sample_time time) { received.push_back(time.value); });
    task.arm(time_point{});
    task.run(time_point{} + 37ms);
    REQUIRE(received == std::vector<seconds>{seconds{37ms}});
    REQUIRE(task.due() == time_point{} + 40ms);
    constexpr auto late = std::chrono::hours{1'000'000} + 37ms;
    task.run(time_point{} + late);
    REQUIRE(received == std::vector<seconds>{seconds{37ms}, seconds{late}});
    REQUIRE(task.due() == time_point{} + std::chrono::hours{1'000'000} + 40ms);
}

TEST_CASE("a one-shot task fires once", "[scheduler][task]")
{
    scheduler loop(inline_workers, dictating());
    std::uint32_t fires    = 0;
    const task_handle task = loop.simulation_strand().after(step_period{seconds{0.010}}, [&fires] { ++fires; });
    REQUIRE(task.valid());
    REQUIRE(task.active());
    advance_clock(loop, std::chrono::milliseconds{10});
    REQUIRE(task.valid());
    REQUIRE_FALSE(task.active());
    advance_clock(loop, std::chrono::milliseconds{10});
    REQUIRE(fires == 1);
}
