#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>

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

// Where the handler had got to when the cancellation arrived, kept outside the handler so a case can
// read it while that handler is still inside its body.
struct held_handler
{
    std::atomic<bool> entered;
    std::atomic<bool> left;
};

auto blocking(std::shared_ptr<held_handler> state, std::promise<void> &inside, std::shared_future<void> released)
{
    return [state = std::move(state), &inside, released](step_delta)
    {
        if(state->entered.exchange(true))
            return;
        inside.set_value();
        released.wait_for(2s);
        state->left = true;
    };
}

}

TEST_CASE("a handler cancelling a task running on another strand returns before that handler does", "[scheduler][cancellation]")
{
    scheduler loop(2);
    auto state          = std::make_shared<held_handler>();
    const strand host   = *loop.make_strand();
    const strand caller = *loop.make_strand();
    std::promise<void> inside, release, returned;
    std::shared_future<void> released = release.get_future().share();
    task_handle running               = host.every(every_step, overrun::drop, blocking(state, inside, released));

    REQUIRE(inside.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(caller.post(
                          [&running, &returned]
                          {
                              running.cancel();
                              returned.set_value();
                          })
                    .has_value());

    REQUIRE(returned.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE_FALSE(state->left.load());

    release.set_value();
}

TEST_CASE("a join asked for from inside a handler is refused and marks nothing", "[scheduler][cancellation]")
{
    scheduler loop(inline_workers, dictating());
    task_handle registered = loop.simulation_strand().after(step_period{10ms}, [] {});
    praxis::expected<void, rejection> answered;

    REQUIRE(loop.make_strand()->post([&] { answered = registered.join(); }).has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE_FALSE(answered.has_value());
    REQUIRE(answered.error() == rejection::reentrant_join);
    REQUIRE(answered.error() != rejection::reentrant_drain);
    REQUIRE(registered.valid());
    REQUIRE(registered.active());
}
