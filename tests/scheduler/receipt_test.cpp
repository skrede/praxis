#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <vector>
#include <cstdint>
#include <utility>

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

// What the handler captures, so that a cancellation returning too early is a read of freed state
// rather than a missing assertion.
struct captured_state
{
    std::atomic<bool> inside;
    std::atomic<bool> left;
};

auto holding(std::shared_ptr<captured_state> state, std::promise<void> &entered, std::shared_future<void> released)
{
    return [state = std::move(state), &entered, released](step_delta)
    {
        if(state->inside.exchange(true))
            return;
        entered.set_value();
        released.wait_for(2s);
        state->left = true;
    };
}

}

TEST_CASE("joining waits out the invocation already inside the handler and cancelling does not", "[scheduler][receipt]")
{
    scheduler loop(2);
    auto waited         = std::make_shared<captured_state>();
    auto marked         = std::make_shared<captured_state>();
    const strand second = *loop.make_strand();
    std::promise<void> waited_entered, marked_entered, release;
    std::shared_future<void> released = release.get_future().share();
    task_handle running               = loop.simulation_strand().every(every_step, overrun::drop, holding(waited, waited_entered, released));
    task_handle discarded             = second.every(every_step, overrun::drop, holding(marked, marked_entered, released));

    REQUIRE(waited_entered.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(marked_entered.get_future().wait_for(2s) == std::future_status::ready);
    REQUIRE(running.active());

    discarded.cancel();
    REQUIRE_FALSE(marked->left.load());

    std::future<praxis::expected<void, rejection>> waiting = std::async(std::launch::async, [&] { return running.join(); });
    REQUIRE(waiting.wait_for(50ms) == std::future_status::timeout);
    REQUIRE_FALSE(waited->left.load());
    release.set_value();
    REQUIRE(waiting.wait_for(2s) == std::future_status::ready);
    REQUIRE(waiting.get().has_value());
    REQUIRE(waited->left.load());
}

TEST_CASE("a stopped pool still reports a handler that is running", "[scheduler][receipt]")
{
    scheduler loop(1);
    auto state = std::make_shared<captured_state>();
    std::promise<void> entered, release;
    std::shared_future<void> released = release.get_future().share();
    const task_handle running         = loop.simulation_strand().every(every_step, overrun::drop, holding(state, entered, released));

    REQUIRE(entered.get_future().wait_for(2s) == std::future_status::ready);
    std::future<void> stopped = std::async(std::launch::async, [&] { loop.stop(); });
    REQUIRE(stopped.wait_for(2s) == std::future_status::ready);
    REQUIRE(running.active());
    release.set_value();
}

TEST_CASE("a handler cancelling its own task stops it after the invocation it is inside", "[scheduler][receipt]")
{
    scheduler loop(inline_workers, dictating());
    std::uint32_t calls = 0;
    task_handle task;
    task = loop.simulation_strand().every(every_step, overrun::drop,
                                          [&](step_delta)
                                          {
                                              ++calls;
                                              task.cancel();
                                          });
    REQUIRE(loop.step());
    REQUIRE_FALSE(loop.step());
    REQUIRE(calls == 1);
}

TEST_CASE("receipts outliving their scheduler cancel nothing", "[scheduler][receipt]")
{
    std::vector<task_handle> held;
    {
        auto loop        = std::make_unique<scheduler>(inline_workers, dictating());
        const strand sim = loop->simulation_strand();
        for(std::uint32_t index = 0; index < 3; ++index)
            held.push_back(sim.after(step_period{10ms}, [] {}));
        for(const task_handle &receipt : held)
        {
            REQUIRE(receipt.valid());
            REQUIRE(receipt.active());
        }
    }

    for(const task_handle &receipt : held)
    {
        REQUIRE_FALSE(receipt.valid());
        REQUIRE_FALSE(receipt.active());
    }
    held.clear();
    REQUIRE(held.empty());
}

TEST_CASE("a receipt moved out of a dead scheduler stays inert", "[scheduler][receipt]")
{
    task_handle moved;
    {
        scheduler loop(inline_workers, dictating());
        task_handle receipt = loop.simulation_strand().after(step_period{10ms}, [] {});
        REQUIRE(receipt.active());
        moved = std::move(receipt);
        REQUIRE(moved.active());
    }
    REQUIRE_FALSE(moved.valid());
    moved.cancel();
    REQUIRE_FALSE(moved.active());
}
