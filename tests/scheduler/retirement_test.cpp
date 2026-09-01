#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <future>
#include <vector>

using namespace praxis::scheduler;

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

}

TEST_CASE("a post to a strand whose retirement has been asked for is refused rather than dropped", "[scheduler][retirement]")
{
    scheduler loop(inline_workers, dictating());
    const praxis::expected<strand, rejection> made = loop.make_strand();

    REQUIRE(made.has_value());

    int ran = 0;
    REQUIRE(made->post([&ran] { ++ran; }).has_value());

    bool acknowledged = false;
    REQUIRE(loop.retire_strand(*made, [&acknowledged] { acknowledged = true; }).has_value());

    const praxis::expected<void, rejection> late = made->post([&ran] { ran += 100; });

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::strand_retired);

    REQUIRE(loop.drain().has_value());
    REQUIRE(ran == 1);
    REQUIRE(acknowledged);
}

TEST_CASE("the acknowledgment runs on the retiring strand and behind everything it already held", "[scheduler][retirement]")
{
    scheduler loop(inline_workers, dictating());
    const strand made = *loop.make_strand();

    std::vector<int> order;
    bool on_the_strand = false;
    for(int index = 0; index < 3; ++index)
        REQUIRE(made.post([&order, index] { order.push_back(index); }).has_value());

    REQUIRE(loop.retire_strand(made,
                               [&order, &on_the_strand, made]
                               {
                                   on_the_strand = made.running_here();
                                   order.push_back(9);
                               })
                    .has_value());

    REQUIRE(loop.drain().has_value());
    REQUIRE(order == std::vector<int>{0, 1, 2, 9});
    REQUIRE(on_the_strand);
}

TEST_CASE("a retired strand runs no further deadline and a second retirement is refused", "[scheduler][retirement]")
{
    scheduler loop(inline_workers, dictating());
    const strand made = *loop.make_strand();

    int ticks              = 0;
    const task_handle tick = made.every(every_step, overrun::drop, [&ticks](step_delta) { ++ticks; });

    REQUIRE(tick.valid());
    REQUIRE(loop.step());
    REQUIRE(ticks == 1);

    REQUIRE(loop.retire_strand(made, nullptr).has_value());

    const praxis::expected<void, rejection> again = loop.retire_strand(made, nullptr);

    REQUIRE_FALSE(again.has_value());
    REQUIRE(again.error() == rejection::strand_retired);

    REQUIRE(loop.drain().has_value());
    REQUIRE(ticks == 1);
}

TEST_CASE("retiring an unknown strand is refused and no other strand is touched", "[scheduler][retirement]")
{
    scheduler loop(inline_workers, dictating());

    const praxis::expected<void, rejection> nowhere = loop.retire_strand(strand(), nullptr);

    REQUIRE_FALSE(nowhere.has_value());
    REQUIRE(nowhere.error() == rejection::unknown_strand);

    int ran = 0;
    REQUIRE(loop.simulation_strand().post([&ran] { ++ran; }).has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(ran == 1);
}

TEST_CASE("a stopped scheduler refuses a retirement rather than taking an acknowledgment it cannot run", "[scheduler][retirement]")
{
    scheduler loop(inline_workers, dictating());
    const strand made = *loop.make_strand();

    loop.stop();

    bool acknowledged                               = false;
    const praxis::expected<void, rejection> refused = loop.retire_strand(made, [&acknowledged] { acknowledged = true; });

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == rejection::scheduler_stopped);
    REQUIRE_FALSE(acknowledged);
}

TEST_CASE("retirement is posted and acknowledged, and the asking thread waits for none of it", "[scheduler][retirement]")
{
    scheduler loop(1);
    const strand sim = loop.simulation_strand();

    std::promise<void> entered;
    std::promise<void> release;
    std::promise<void> acknowledged;
    std::future<void> inside = entered.get_future();
    std::future<void> held   = release.get_future();
    std::future<void> done   = acknowledged.get_future();

    // The handler is bounded rather than open-ended, so a retirement that joined instead of posting
    // would return late and fail the assertion below rather than hang the binary.
    REQUIRE(sim.post(
                       [&entered, &held]
                       {
                           entered.set_value();
                           held.wait_for(std::chrono::seconds(2));
                       })
                    .has_value());

    REQUIRE(inside.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
    REQUIRE(loop.retire_strand(sim, [&acknowledged] { acknowledged.set_value(); }).has_value());
    REQUIRE(done.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);

    release.set_value();

    REQUIRE(done.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
}
