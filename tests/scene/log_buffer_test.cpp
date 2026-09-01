#include "praxis/scene/log_buffer.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cstddef>
#include <utility>
#include <unordered_set>
#include <initializer_list>

namespace {

using praxis::scene::log_buffer;
using praxis::scene::log_entry;
using praxis::scene::severity;

std::string numbered(std::size_t index)
{
    return "message " + std::to_string(index);
}

void push_numbered(log_buffer &buffer, std::size_t first, std::size_t count)
{
    for(std::size_t i = 0; i < count; ++i)
        buffer.push(numbered(first + i), severity::info);
}

std::vector<std::string> texts(const std::vector<log_entry> &entries)
{
    std::vector<std::string> out;
    out.reserve(entries.size());
    for(const log_entry &entry : entries)
        out.push_back(entry.text);

    return out;
}

std::vector<std::string> numbered_range(std::size_t first, std::size_t count)
{
    std::vector<std::string> out;
    out.reserve(count);
    for(std::size_t i = 0; i < count; ++i)
        out.push_back(numbered(first + i));

    return out;
}

std::string written_by(std::size_t writer, std::size_t index)
{
    return "writer " + std::to_string(writer) + " entry " + std::to_string(index);
}

// The drain after the flag falls is what makes the accounting exact: without it the entries pushed
// between the last drain and the join would be neither drained nor dropped.
std::thread start_draining(log_buffer &buffer, std::atomic<bool> &writing, std::vector<log_entry> &collected)
{
    return std::thread(
            [&]
            {
                while(writing.load())
                    for(log_entry &entry : buffer.drain())
                        collected.push_back(std::move(entry));
                for(log_entry &entry : buffer.drain())
                    collected.push_back(std::move(entry));
            });
}

// One thread drains while the others push texts no two of which are equal, so nothing coalesces and
// every drained entry stands for exactly one push.
std::vector<log_entry> drain_while_writing(log_buffer &buffer, std::size_t writers, std::size_t per_writer)
{
    std::vector<log_entry> collected;
    std::atomic<bool> writing(true);
    std::thread reader = start_draining(buffer, writing, collected);

    std::vector<std::thread> pushers;
    for(std::size_t writer = 0; writer < writers; ++writer)
        pushers.emplace_back(
                [&buffer, writer, per_writer]
                {
                    for(std::size_t index = 0; index < per_writer; ++index)
                        buffer.push(written_by(writer, index), severity::info);
                });

    for(std::thread &pusher : pushers)
        pusher.join();
    writing.store(false);
    reader.join();

    return collected;
}

std::unordered_set<std::string> every_written_text(std::size_t writers, std::size_t per_writer)
{
    std::unordered_set<std::string> expected;
    for(std::size_t writer = 0; writer < writers; ++writer)
        for(std::size_t index = 0; index < per_writer; ++index)
            expected.insert(written_by(writer, index));

    return expected;
}

constexpr std::size_t writers    = 8;
constexpr std::size_t per_writer = 2000;

}

TEST_CASE("fewer entries than the capacity drain in the order they were pushed", "[scene]")
{
    log_buffer buffer(8);
    push_numbered(buffer, 0, 5);

    REQUIRE(texts(buffer.drain()) == numbered_range(0, 5));
    REQUIRE(buffer.dropped() == 0);
}

TEST_CASE("the ring keeps the newest entries and drops the oldest", "[scene]")
{
    constexpr std::size_t capacity = 4;

    SECTION("exactly at the capacity nothing is dropped")
    {
        log_buffer buffer(capacity);
        push_numbered(buffer, 0, capacity);

        REQUIRE(texts(buffer.drain()) == numbered_range(0, capacity));
        REQUIRE(buffer.dropped() == 0);
    }

    SECTION("one past the capacity drops the oldest and keeps the rest")
    {
        log_buffer buffer(capacity);
        push_numbered(buffer, 0, capacity + 1);

        REQUIRE(texts(buffer.drain()) == numbered_range(1, capacity));
        REQUIRE(buffer.dropped() == 1);
    }

    SECTION("two wraps and one more keep the newest capacity-worth")
    {
        log_buffer buffer(capacity);
        push_numbered(buffer, 0, 2 * capacity + 1);

        REQUIRE(texts(buffer.drain()) == numbered_range(capacity + 1, capacity));
        REQUIRE(buffer.dropped() == capacity + 1);
    }
}

TEST_CASE("a message repeated in a row is one entry carrying its repeat count", "[scene]")
{
    log_buffer buffer(8);

    SECTION("the same text and severity coalesce")
    {
        buffer.push("the composition was refused", severity::warning);
        buffer.push("the composition was refused", severity::warning);
        buffer.push("the composition was refused", severity::warning);

        const std::vector<log_entry> drained = buffer.drain();

        REQUIRE(drained.size() == 1);
        REQUIRE(drained.front().repeats == 3);
    }

    SECTION("the same text at another severity is a separate entry")
    {
        buffer.push("the composition was refused", severity::warning);
        buffer.push("the composition was refused", severity::error);

        const std::vector<log_entry> drained = buffer.drain();

        REQUIRE(drained.size() == 2);
        REQUIRE(drained.front().repeats == 1);
        REQUIRE(drained.back().repeats == 1);
    }

    SECTION("a different message in between starts a new entry")
    {
        buffer.push("the composition was refused", severity::warning);
        buffer.push("the composition was accepted", severity::info);
        buffer.push("the composition was refused", severity::warning);

        const std::vector<log_entry> drained = buffer.drain();

        REQUIRE(texts(drained) == std::vector<std::string>{"the composition was refused", "the composition was accepted", "the composition was refused"});
        REQUIRE(drained.back().repeats == 1);
    }
}

TEST_CASE("a message repeated far past the capacity still occupies one entry", "[scene]")
{
    log_buffer buffer(4);
    for(std::size_t i = 0; i < 10000; ++i)
        buffer.push("the composition was refused", severity::warning);

    const std::vector<log_entry> drained = buffer.drain();

    REQUIRE(drained.size() == 1);
    REQUIRE(drained.front().repeats == 10000);
    REQUIRE(buffer.dropped() == 0);
}

TEST_CASE("draining empties the ring", "[scene]")
{
    log_buffer buffer(8);
    push_numbered(buffer, 0, 3);

    REQUIRE(buffer.drain().size() == 3);
    REQUIRE(buffer.drain().empty());
    REQUIRE(buffer.dropped() == 0);
}

TEST_CASE("a ring wide enough for the whole run loses nothing under concurrent pushes", "[scene]")
{
    log_buffer buffer(writers * per_writer);
    const std::vector<log_entry> collected   = drain_while_writing(buffer, writers, per_writer);
    std::unordered_set<std::string> expected = every_written_text(writers, per_writer);

    REQUIRE(buffer.dropped() == 0);
    REQUIRE(collected.size() == writers * per_writer);
    for(const log_entry &entry : collected)
    {
        REQUIRE(entry.repeats == 1);
        REQUIRE(expected.erase(entry.text) == 1);
    }
    REQUIRE(expected.empty());
}

TEST_CASE("a ring narrower than the run accounts for every push as drained or dropped", "[scene]")
{
    log_buffer buffer(16);
    const std::vector<log_entry> collected         = drain_while_writing(buffer, writers, per_writer);
    const std::unordered_set<std::string> expected = every_written_text(writers, per_writer);

    std::size_t counted = 0;
    for(const log_entry &entry : collected)
    {
        REQUIRE(entry.repeats == 1);
        REQUIRE(expected.count(entry.text) == 1);
        counted += entry.repeats;
    }

    REQUIRE(counted + buffer.dropped() == writers * per_writer);
}

// The level is held by the logger, which every sink hangs off, so this case leaves it where it found
// it rather than deciding for whatever runs next.
TEST_CASE("the reporting level answers what was last set", "[scene]")
{
    const severity found = praxis::scene::reporting_level();

    SECTION("nothing set yet reads as the level an operator sees without changing anything")
    {
        REQUIRE(found == severity::info);
    }

    SECTION("every published level is answered back")
    {
        for(severity level : {severity::debug, severity::info, severity::warning, severity::error})
        {
            praxis::scene::set_reporting_level(level);
            REQUIRE(praxis::scene::reporting_level() == level);
        }
    }

    praxis::scene::set_reporting_level(found);
}

TEST_CASE("installing the window sink adds to the sinks already present", "[scene]")
{
    // The default logger outlives this case, so a buffer the installed sink refers to must too.
    static const std::shared_ptr<log_buffer> first  = std::make_shared<log_buffer>(8);
    static const std::shared_ptr<log_buffer> second = std::make_shared<log_buffer>(8);

    const std::size_t after_first  = praxis::scene::install_log_sink(first);
    const std::size_t after_second = praxis::scene::install_log_sink(second);

    REQUIRE(after_first >= 2);
    REQUIRE(after_second == after_first + 1);
}
