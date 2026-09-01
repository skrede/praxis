#include "praxis/scheduler/scheduler.h"
#include "praxis/scheduler/ownership.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <future>
#include <string>
#include <cstdint>
#include <stdexcept>

using namespace praxis::scheduler;

namespace {

// Neither copyable nor movable, so a wrapper holding one has to build it where it stands.
class ledger
{
public:
    ledger(int start, int step)
            : m_total(start)
            , m_step(step)
    {
    }

    ledger(const ledger &)            = delete;
    ledger(ledger &&)                 = delete;
    ledger &operator=(const ledger &) = delete;
    ledger &operator=(ledger &&)      = delete;

    ~ledger() = default;

    int total() const
    {
        return m_total;
    }

    void advance()
    {
        m_total += m_step;
    }

private:
    int m_total;
    int m_step;
};

// What one handler entered from inside another handler answers, so the two worker counts are
// comparable field by field rather than by two sets of separate assertions.
struct nesting
{
    bool admitted;
    bool serviced;
    bool outer_in_inner;
    bool inner_in_inner;
    bool other_in_inner;
    bool outer_after;
    bool inner_after;
    bool outer_outside;

    bool operator==(const nesting &) const = default;
};

// The outer handler's own body: it admits the inner one, services it on the thread it is already
// running on, and reads both strands on either side of that inner scope.
void enter_nested(scheduler &loop, const strand &outer, const strand &inner, const strand &other, nesting &seen)
{
    seen.admitted    = inner.post(
                                    [&]
                                    {
                                     seen.outer_in_inner = outer.running_here();
                                     seen.inner_in_inner = inner.running_here();
                                     seen.other_in_inner = other.running_here();
                                    })
                               .has_value();
    seen.serviced    = loop.step();
    seen.outer_after = outer.running_here();
    seen.inner_after = inner.running_here();
}

// Main is the inner strand because no worker ever services it, so whoever calls step() from inside
// the outer handler is the thread that runs the inner one at every worker count.
nesting nest(std::uint32_t workers)
{
    scheduler loop(workers);
    const strand inner = loop.main_strand();
    const strand outer = *loop.make_strand();
    const strand other = *loop.make_strand();
    nesting seen{false, false, false, false, false, false, false, false};
    std::promise<void> finished;
    std::future<void> done = finished.get_future();

    const praxis::expected<void, rejection> admitted = outer.post(
            [&]
            {
                enter_nested(loop, outer, inner, other, seen);
                finished.set_value();
            });
    REQUIRE(admitted.has_value());
    if(workers == inline_workers)
        REQUIRE(loop.drain().has_value());
    else
        REQUIRE(done.wait_for(std::chrono::seconds(5)) == std::future_status::ready);

    seen.outer_outside = outer.running_here();
    return seen;
}

}

TEST_CASE("a strand reports itself as running here only inside a handler of its own", "[scheduler][ownership]")
{
    scheduler loop(inline_workers);
    const strand one = *loop.make_strand();
    const strand two = *loop.make_strand();

    bool one_inside = false;
    bool two_inside = false;

    REQUIRE(one.post(
                       [&]
                       {
                           one_inside = one.running_here();
                           two_inside = two.running_here();
                       })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(one_inside);
    REQUIRE_FALSE(two_inside);
    REQUIRE_FALSE(one.running_here());
    REQUIRE_FALSE(two.running_here());
    REQUIRE_FALSE(strand().running_here());
}

TEST_CASE("a handler entered from inside another handler leaves both strands reported as running here", "[scheduler][ownership]")
{
    const nesting quiet = nest(inline_workers);

    CHECK(quiet.admitted);
    CHECK(quiet.serviced);
    CHECK(quiet.outer_in_inner);
    CHECK(quiet.inner_in_inner);
    CHECK_FALSE(quiet.other_in_inner);
    CHECK(quiet.outer_after);
    CHECK_FALSE(quiet.inner_after);
    CHECK_FALSE(quiet.outer_outside);
}

TEST_CASE("the nesting answers do not depend on the worker count", "[scheduler][ownership]")
{
    REQUIRE(nest(2) == nest(inline_workers));
}

TEST_CASE("the gate posts rather than running on whoever calls it", "[scheduler][ownership]")
{
    scheduler loop(inline_workers);
    const strand one = *loop.make_strand();
    strand_owned<ledger> book(one, 5, 3);

    int reported = 0;
    int calls    = 0;

    const praxis::expected<void, rejection> outside = book.with(
            [&calls, &reported](ledger &held)
            {
                ++calls;
                held.advance();
                reported = held.total();
            });

    REQUIRE(outside.has_value());
    REQUIRE(calls == 0);

    int calls_when_the_gate_returned = -1;
    bool nested_admitted             = false;

    REQUIRE(one.post(
                       [&]
                       {
                           const praxis::expected<void, rejection> inside = book.with(
                                   [&calls](ledger &held)
                                   {
                                       ++calls;
                                       held.advance();
                                   });
                           nested_admitted              = inside.has_value();
                           calls_when_the_gate_returned = calls;
                       })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(nested_admitted);
    REQUIRE(calls_when_the_gate_returned == 1);
    REQUIRE(calls == 2);
    REQUIRE(reported == 8);
}

TEST_CASE("the gate refuses rather than throwing when the strand admits nothing further", "[scheduler][ownership]")
{
    scheduler loop(inline_workers);
    const strand retiring = *loop.make_strand();
    const strand stopping = *loop.make_strand();
    strand_owned<int> counted(retiring, 0);
    strand_owned<int> halted(stopping, 0);

    REQUIRE(loop.retire_strand(retiring, nullptr).has_value());

    const praxis::expected<void, rejection> refused = counted.with([](int &value) { ++value; });

    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == rejection::strand_retired);

    loop.stop();

    const praxis::expected<void, rejection> late = halted.with([](int &value) { ++value; });

    REQUIRE_FALSE(late.has_value());
    REQUIRE(late.error() == rejection::scheduler_stopped);
}

TEST_CASE("touching an object from a strand that does not own it throws and names both", "[scheduler][ownership]")
{
    scheduler loop(inline_workers);
    const strand one = *loop.make_strand();
    const strand two = *loop.make_strand();

    REQUIRE_THROWS_AS(require_strand(one, "the rendered robot"), std::logic_error);

    std::string reported;
    bool refused_on_two = false;
    bool refused_on_one = false;

    REQUIRE(two.post(
                       [&]
                       {
                           try
                           {
                               require_strand(one, "the rendered robot");
                           }
                           catch(const std::logic_error &refusal)
                           {
                               refused_on_two = true;
                               reported       = refusal.what();
                           }
                       })
                    .has_value());
    REQUIRE(one.post(
                       [&]
                       {
                           try
                           {
                               require_strand(one, "the rendered robot");
                           }
                           catch(const std::logic_error &)
                           {
                               refused_on_one = true;
                           }
                       })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    REQUIRE(refused_on_two);
    REQUIRE_FALSE(refused_on_one);
    REQUIRE(reported.find("the rendered robot") != std::string::npos);
    REQUIRE(reported.find("strand " + std::to_string(static_cast<std::uint32_t>(one.id()))) != std::string::npos);
}
