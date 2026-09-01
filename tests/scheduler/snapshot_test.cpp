#include "praxis/scheduler/snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstddef>
#include <cstdint>
#include <optional>

using namespace praxis::scheduler;

namespace {

constexpr std::size_t angles = 32;

// Every field is derived from the sequence number, so a value assembled out of two publications
// fails the derivation while anything the publisher wrote whole satisfies it.
struct marked_sample
{
    std::uint64_t sequence;
    std::uint64_t echo;
    std::array<double, angles> angle;
};

// A default the language would not have produced on its own, so reading it is evidence the type's
// own default constructor ran rather than that the storage happened to be zero.
class dial
{
public:
    dial()
            : m_value(-1.0)
    {
    }

    double value() const
    {
        return m_value;
    }

private:
    double m_value;
};

double derived(std::uint64_t sequence, std::size_t index)
{
    return static_cast<double>(sequence) + static_cast<double>(index) / 1024.0;
}

marked_sample marked(std::uint64_t sequence)
{
    marked_sample sample{sequence, sequence, {}};
    for(std::size_t i = 0; i < angles; ++i)
        sample.angle[i] = derived(sequence, i);

    return sample;
}

bool whole(const marked_sample &sample)
{
    if(sample.echo != sample.sequence)
        return false;
    for(std::size_t i = 0; i < angles; ++i)
        if(sample.angle[i] != derived(sample.sequence, i))
            return false;

    return true;
}

// The run length, and the floor on how much of it the reader has to have witnessed: at least a
// million publications offered at the highest rate one thread produces them, against a reader that
// never pauses, and a hundred thousand changes of value observed. Without the floor a run whose
// reader started after the last publication would pass having examined one value. The floor cannot
// be a fraction of a fixed count: how far the publisher gets before the reader's thread first runs
// is the scheduler's business, and a lock handed back is not handed over -- an unfair lock lets the
// publisher retake it before a waiting reader wakes, for as long as the publisher keeps asking. So
// the flood is followed by publications offered one at a time, the publisher standing clear of the
// lock until the reader has witnessed each, under a patience that fits the suite's own limit on
// this binary.
constexpr std::uint64_t publications  = 1000000;
constexpr std::uint64_t least_changes = 100000;
constexpr std::chrono::seconds patience{7};

struct observation_report
{
    std::uint64_t observations;
    std::uint64_t changes;
    std::uint64_t torn;
    std::uint64_t regressed;
};

observation_report read_while_publishing(snapshot_reader<marked_sample> view, const std::atomic<bool> &publishing, std::atomic<std::uint64_t> &witnessed)
{
    observation_report report{0, 0, 0, 0};
    std::uint64_t previous = 0;
    while(publishing.load(std::memory_order_relaxed))
    {
        const marked_sample seen = view.read();
        ++report.observations;
        report.changes += seen.sequence != previous ? std::uint64_t{1} : std::uint64_t{0};
        report.torn += whole(seen) ? std::uint64_t{0} : std::uint64_t{1};
        report.regressed += seen.sequence < previous ? std::uint64_t{1} : std::uint64_t{0};
        previous = seen.sequence;
        witnessed.store(report.changes, std::memory_order_relaxed);
    }

    return report;
}

observation_report publish_against_a_reader()
{
    snapshot_publisher<marked_sample> source;
    source.publish(marked(1));

    std::atomic<bool> publishing(true);
    std::atomic<std::uint64_t> witnessed(0);
    observation_report report{0, 0, 0, 0};
    std::thread watcher([&] { report = read_while_publishing(source.reader(), publishing, witnessed); });

    std::uint64_t sequence = 1;
    while(sequence < publications)
        source.publish(marked(++sequence));

    // One witnessed change per publication from here: the value is published and the publisher
    // then stands clear of the lock until the reader has seen it, so a lock that favors whoever
    // asks last cannot keep the reader out, and reaching the floor is a count of round trips
    // rather than a race the scheduler referees.
    const std::chrono::steady_clock::time_point give_up = std::chrono::steady_clock::now() + patience;
    bool patient                                        = true;
    while(patient && witnessed.load(std::memory_order_relaxed) < least_changes)
    {
        const std::uint64_t seen = witnessed.load(std::memory_order_relaxed);
        source.publish(marked(++sequence));
        while(patient && witnessed.load(std::memory_order_relaxed) == seen)
        {
            std::this_thread::yield();
            patient = std::chrono::steady_clock::now() < give_up;
        }
    }
    publishing.store(false);
    watcher.join();

    return report;
}

}

TEST_CASE("a reader taken before the first publication reads the default-constructed value", "[scheduler]")
{
    snapshot_publisher<marked_sample> aggregate;
    const marked_sample seen = aggregate.reader().read();

    REQUIRE(seen.sequence == 0);
    REQUIRE(seen.echo == 0);
    REQUIRE(seen.angle == std::array<double, angles>{});

    snapshot_publisher<dial> constructed;

    REQUIRE(constructed.reader().read().value() == -1.0);
}

TEST_CASE("a published value is what a reader taken from the publisher returns", "[scheduler]")
{
    snapshot_publisher<marked_sample> source;
    const snapshot_reader<marked_sample> view = source.reader();

    source.publish(marked(7));
    REQUIRE(view.read().sequence == 7);
    REQUIRE(whole(view.read()));

    source.publish(marked(8));
    REQUIRE(view.read().sequence == 8);
}

TEST_CASE("two readers taken from one publisher observe the same value", "[scheduler]")
{
    snapshot_publisher<marked_sample> source;
    const snapshot_reader<marked_sample> first  = source.reader();
    const snapshot_reader<marked_sample> second = source.reader();

    source.publish(marked(11));
    REQUIRE(first.read().sequence == second.read().sequence);

    source.publish(marked(12));
    REQUIRE(first.read().sequence == 12);
    REQUIRE(second.read().sequence == 12);
}

TEST_CASE("a reader outlives the publisher it was taken from", "[scheduler]")
{
    std::optional<snapshot_reader<marked_sample>> view;
    {
        snapshot_publisher<marked_sample> source;
        view = source.reader();
        source.publish(marked(5));
    }

    REQUIRE(view->read().sequence == 5);
    REQUIRE(whole(view->read()));
}

TEST_CASE("a publisher outlives every reader taken from it", "[scheduler]")
{
    snapshot_publisher<marked_sample> source;
    {
        const snapshot_reader<marked_sample> view = source.reader();
        source.publish(marked(3));

        REQUIRE(view.read().sequence == 3);
    }

    source.publish(marked(4));
    REQUIRE(source.reader().read().sequence == 4);
}

TEST_CASE("a read concurrent with a publication never observes a value assembled out of two", "[scheduler]")
{
    const observation_report report = publish_against_a_reader();

    CHECK(report.torn == 0);
    CHECK(report.regressed == 0);
    REQUIRE(report.observations > 0);
    REQUIRE(report.changes >= least_changes);
}
