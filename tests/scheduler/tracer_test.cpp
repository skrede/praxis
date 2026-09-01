#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <memory>
#include <utility>
#include <optional>
#include <type_traits>

using namespace praxis::scheduler;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

// Every case reads its own time, so what a policy does with an interval is dictated rather than
// waited for.
clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

auto move_only_closure()
{
    return [held = std::make_unique<int>(1)] { REQUIRE(*held == 1); };
}

}

TEST_CASE("a strand is a copyable handle and posted work is a move-only callable", "[scheduler]")
{
    using held = praxis::detail::move_only_function<void()>;

    STATIC_REQUIRE(std::is_default_constructible_v<strand>);
    STATIC_REQUIRE(std::is_copy_constructible_v<strand>);
    STATIC_REQUIRE(std::is_copy_assignable_v<strand>);
    STATIC_REQUIRE(std::is_constructible_v<held, decltype(move_only_closure())>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<held>);
    REQUIRE_FALSE(strand().valid());
}

TEST_CASE("one advance runs one posted unit and the next reports quiescence", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    int ran    = 0;
    auto owned = std::make_unique<int>(1);
    REQUIRE(main.post([&ran, held = std::move(owned)] { ran += *held; }).has_value());

    REQUIRE(loop.step());
    REQUIRE(ran == 1);

    REQUIRE_FALSE(loop.step());
    REQUIRE(ran == 1);
}

TEST_CASE("a drain runs what is queued and leaves nothing behind", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    int ran = 0;
    for(int index = 0; index < 3; ++index)
        REQUIRE(main.post([&ran] { ++ran; }).has_value());

    const praxis::expected<void, rejection> drained = loop.drain();

    REQUIRE(drained.has_value());
    REQUIRE(ran == 3);
    REQUIRE_FALSE(loop.step());
}

TEST_CASE("a handler runs on the strand it was posted to", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    bool inside = false;
    REQUIRE(main.post([&inside, main] { inside = main.running_here(); }).has_value());

    REQUIRE_FALSE(main.running_here());
    REQUIRE(loop.step());
    REQUIRE(inside);
    REQUIRE_FALSE(main.running_here());
}

TEST_CASE("a handler that stops the scheduler returns the run", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    int frames              = 0;
    int queued              = 0;
    const task_handle frame = main.every(every_step, overrun::drop,
                                         [&loop, &main, &frames, &queued](step_delta)
                                         {
                                             ++frames;
                                             REQUIRE(frames == 1);
                                             REQUIRE(main.post([&queued] { ++queued; }).has_value());
                                             loop.stop();
                                         });

    REQUIRE(frame.valid());
    loop.run();

    REQUIRE(frames == 1);
    REQUIRE(queued == 0);
}

TEST_CASE("a stepped task is handed the interval that actually passed", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    seconds measured{0.0};
    const task_handle tick = main.every(step_period{seconds{0.5}}, overrun::drop, [&measured](step_delta delta) { measured = delta.value; });
    dictated += std::chrono::milliseconds(1500);

    REQUIRE(loop.step());
    REQUIRE(measured.count() > 1.4);
    REQUIRE(measured.count() < 1.6);
}

TEST_CASE("a catch-up task decomposes the interval into the period it was given", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    int ticks = 0;
    seconds each{0.0};
    const task_handle tick = main.every(step_period{seconds{0.5}}, overrun::catch_up,
                                        [&ticks, &each](step_delta delta)
                                        {
                                            ++ticks;
                                            each = delta.value;
                                        });
    dictated += std::chrono::milliseconds(1500);

    REQUIRE(loop.step());
    REQUIRE(ticks == 3);
    REQUIRE(each.count() > 0.4);
    REQUIRE(each.count() < 0.6);
}

TEST_CASE("cancelling the receipt retires the task", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    int ticks = 0;
    {
        const task_handle tick = main.every(every_step, overrun::drop, [&ticks](step_delta) { ++ticks; });
        REQUIRE(loop.step());
    }

    REQUIRE(ticks == 1);
    REQUIRE_FALSE(loop.step());
    REQUIRE(ticks == 1);
}

TEST_CASE("a post names what it refuses", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());

    const praxis::expected<void, rejection> unknown = strand().post([] {});
    const praxis::expected<void, rejection> empty   = loop.main_strand().post(praxis::detail::move_only_function<void()>());

    REQUIRE_FALSE(unknown.has_value());
    REQUIRE(unknown.error() == rejection::unknown_strand);
    REQUIRE_FALSE(empty.has_value());
    REQUIRE(empty.error() == rejection::empty_work);
}

TEST_CASE("a drain asked for from inside a handler is refused", "[scheduler]")
{
    scheduler loop(inline_workers, dictating());
    const strand main = loop.main_strand();

    std::optional<rejection> refused;
    REQUIRE(main.post(
                        [&loop, &refused]
                        {
                            const praxis::expected<void, rejection> inner = loop.drain();
                            if(!inner.has_value())
                                refused = inner.error();
                        })
                    .has_value());

    REQUIRE(loop.step());
    REQUIRE(refused.has_value());
    REQUIRE(*refused == rejection::reentrant_drain);
}
