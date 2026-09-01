#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <vector>
#include <type_traits>

using namespace praxis::scheduler;

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

// What one strand answers to the identical sequence, so four of them can be compared field by field
// rather than by four separate assertions each.
struct round_trip
{
    bool valid;
    bool admitted;
    bool ran;
    bool inside;
    bool outside;
};

round_trip drive(scheduler &loop, strand one)
{
    round_trip seen{one.valid(), false, false, false, false};

    seen.admitted = one.post(
                               [&seen, one]
                               {
                                   seen.ran    = true;
                                   seen.inside = one.running_here();
                               })
                            .has_value();
    REQUIRE(loop.drain().has_value());
    seen.outside = one.running_here();

    return seen;
}

}

TEST_CASE("what make_strand yields and what simulation_strand yields are one type", "[scheduler][strand]")
{
    using made     = std::remove_cvref_t<decltype(*std::declval<scheduler &>().make_strand())>;
    using built_in = std::remove_cvref_t<decltype(std::declval<const scheduler &>().simulation_strand())>;

    STATIC_REQUIRE(std::is_same_v<made, built_in>);
    STATIC_REQUIRE(std::is_same_v<made, strand>);
}

TEST_CASE("every strand is a peer: whatever made it, it round-trips post, run and identity alike", "[scheduler][strand]")
{
    scheduler loop(inline_workers, dictating());

    const praxis::expected<strand, rejection> first  = loop.make_strand();
    const praxis::expected<strand, rejection> second = loop.make_strand();

    REQUIRE(first.has_value());
    REQUIRE(second.has_value());

    const std::vector<strand> every{loop.main_strand(), loop.simulation_strand(), *first, *second};

    for(const strand &one : every)
    {
        const round_trip seen = drive(loop, one);

        REQUIRE(seen.valid);
        REQUIRE(seen.admitted);
        REQUIRE(seen.ran);
        REQUIRE(seen.inside);
        REQUIRE_FALSE(seen.outside);
    }
}

TEST_CASE("a made strand is an identity of its own and answers for no other", "[scheduler][strand]")
{
    scheduler loop(inline_workers, dictating());

    const strand main  = loop.main_strand();
    const strand sim   = loop.simulation_strand();
    const strand first = *loop.make_strand();
    const strand other = *loop.make_strand();

    bool confused = false;
    REQUIRE(first.post([&confused, main, sim, other] { confused = main.running_here() || sim.running_here() || other.running_here(); }).has_value());

    REQUIRE(loop.drain().has_value());
    REQUIRE_FALSE(confused);
}

TEST_CASE("a default-constructed strand reports itself invalid and refuses what it is handed", "[scheduler][strand]")
{
    const strand nowhere;
    const praxis::expected<void, rejection> refused = nowhere.post([] {});

    REQUIRE_FALSE(nowhere.valid());
    REQUIRE_FALSE(nowhere.running_here());
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == rejection::unknown_strand);
}

TEST_CASE("a stopped scheduler makes no further strand and its strands admit nothing", "[scheduler][strand]")
{
    scheduler loop(inline_workers, dictating());
    const strand sim = loop.simulation_strand();

    loop.stop();

    const praxis::expected<strand, rejection> made   = loop.make_strand();
    const praxis::expected<void, rejection> admitted = sim.post([] {});

    REQUIRE_FALSE(made.has_value());
    REQUIRE(made.error() == rejection::scheduler_stopped);
    REQUIRE_FALSE(admitted.has_value());
    REQUIRE(admitted.error() == rejection::scheduler_stopped);
}

TEST_CASE("with no workers every strand is serviced by the caller and a drain runs all of it", "[scheduler][strand]")
{
    scheduler loop(inline_workers, dictating());
    const std::thread::id caller = std::this_thread::get_id();

    int ran       = 0;
    int elsewhere = 0;
    for(const strand &one : {loop.main_strand(), loop.simulation_strand(), *loop.make_strand()})
        for(int index = 0; index < 4; ++index)
            REQUIRE(one.post(
                               [&ran, &elsewhere, caller]
                               {
                                   ++ran;
                                   if(std::this_thread::get_id() != caller)
                                       ++elsewhere;
                               })
                            .has_value());

    REQUIRE(loop.drain().has_value());
    REQUIRE(ran == 12);
    REQUIRE(elsewhere == 0);
    REQUIRE_FALSE(loop.step());
}

TEST_CASE("main is the one strand the workers never service", "[scheduler][strand]")
{
    constexpr int offered = 256;

    scheduler loop(2);
    const strand main = loop.main_strand();
    const strand sim  = loop.simulation_strand();

    const std::thread::id runner = std::this_thread::get_id();

    std::atomic<int> ran{0};
    std::atomic<int> admitted{0};
    std::atomic<int> elsewhere{0};

    // Main's work is offered from a handler a worker is already running, so the workers are awake
    // and scanning when it arrives: what keeps them off it is that main is closed to them, not that
    // they were asleep. The run is ended by a closure posted behind the offer and every count is
    // published before that closure exists, so nothing this reads afterwards is still being written.
    REQUIRE(sim.post(
                       [&loop, &main, &ran, &admitted, &elsewhere, runner]
                       {
                           int taken = 0;
                           for(int index = 0; index < offered; ++index)
                               if(main.post(
                                              [&ran, &elsewhere, runner]
                                              {
                                                  if(std::this_thread::get_id() != runner)
                                                      elsewhere.fetch_add(1, std::memory_order_relaxed);
                                                  ran.fetch_add(1, std::memory_order_relaxed);
                                              })
                                          .has_value())
                                   ++taken;
                           admitted.store(taken, std::memory_order_relaxed);

                           const praxis::expected<void, rejection> ended = main.post([&loop] { loop.stop(); });
                           if(!ended.has_value())
                               loop.stop();
                       })
                    .has_value());

    loop.run();

    REQUIRE(admitted.load() == offered);
    REQUIRE(ran.load() == offered);
    REQUIRE(elsewhere.load() == 0);
}

TEST_CASE("a high-rate task keeps its ticks while the main strand is inside a long handler", "[scheduler][strand]")
{
    constexpr int periods = 8;
    constexpr auto period = std::chrono::milliseconds(10);

    scheduler loop(default_workers(), dictating());
    const strand main = loop.main_strand();
    const strand sim  = loop.simulation_strand();

    std::atomic<int> ticks{0};
    std::atomic<bool> announced{false};
    std::promise<void> owed;
    std::future<void> reached = owed.get_future();

    const task_handle high = sim.every(step_period{seconds{0.010}}, overrun::catch_up,
                                       [&ticks, &announced, &owed](step_delta)
                                       {
                                           if(ticks.fetch_add(1, std::memory_order_relaxed) + 1 >= periods && !announced.exchange(true))
                                               owed.set_value();
                                       });

    int counted             = 0;
    bool delivered          = false;
    const task_handle frame = main.every(every_step, overrun::drop,
                                         [&](step_delta)
                                         {
                                             dictated.store(dictated.load() + periods * period, std::memory_order_release);
                                             delivered = reached.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
                                             counted   = ticks.load(std::memory_order_relaxed);
                                             loop.stop();
                                         });

    REQUIRE(high.valid());
    REQUIRE(frame.valid());
    loop.run();

    REQUIRE(delivered);
    REQUIRE(counted >= periods - 1);
    REQUIRE(counted <= periods + 1);
}

TEST_CASE("a drain asked for from inside a handler on a worker is refused rather than blocked", "[scheduler][strand]")
{
    scheduler loop(1);
    const strand sim = loop.simulation_strand();

    std::promise<rejection> refused;
    std::future<rejection> answered = refused.get_future();

    REQUIRE(sim.post(
                       [&loop, &refused]
                       {
                           const praxis::expected<void, rejection> inner = loop.drain();
                           refused.set_value(inner.has_value() ? rejection::empty_work : inner.error());
                       })
                    .has_value());

    REQUIRE(answered.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    REQUIRE(answered.get() == rejection::reentrant_drain);
}
