#include "praxis/scene/log_buffer.h"
#include "praxis/scene/coverage_report.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/descriptor.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace {

using praxis::capability_view;
using praxis::slot_descriptor;
using praxis::capability_descriptors;
using praxis::scene::log_buffer;
using praxis::scene::log_entry;
using praxis::scene::severity;

int inert_first()
{
    return 0;
}

int inert_second()
{
    return 0;
}

int bound_second()
{
    return 1;
}

// The suite links the core alone, so what is reported on is built here rather than borrowed from an
// extension.
struct sample_ops
{
    int (*first)()  = &inert_first;
    int (*second)() = &inert_second;
};

constexpr std::array sample_descriptors{
        slot_descriptor{"sample.first", [](const void *value) -> bool { return static_cast<const sample_ops *>(value)->first == &inert_first; }},
        slot_descriptor{"sample.second", [](const void *value) -> bool { return static_cast<const sample_ops *>(value)->second == &inert_second; }},
};

constexpr capability_descriptors<sample_ops> described_sample{"sample", sample_descriptors};

// The default logger outlives every case, so the buffer its sink refers to must too.
const std::shared_ptr<log_buffer> &messages()
{
    static const std::shared_ptr<log_buffer> buffer = []
    {
        std::shared_ptr<log_buffer> made = std::make_shared<log_buffer>(praxis::scene::default_log_capacity);
        praxis::scene::install_log_sink(made);

        return made;
    }();

    return buffer;
}

std::vector<log_entry> reported_at(severity level, std::span<const capability_view> views)
{
    const severity found = praxis::scene::reporting_level();

    messages()->drain();
    praxis::scene::set_reporting_level(level);
    praxis::scene::report_default_slots(views);
    praxis::scene::set_reporting_level(found);

    return messages()->drain();
}

std::size_t entries_at(const std::vector<log_entry> &drained, severity level)
{
    std::size_t held = 0;
    for(const log_entry &entry : drained)
        held += entry.level == level ? 1u : 0u;

    return held;
}

bool mentions(const std::vector<log_entry> &drained, std::string_view text)
{
    for(const log_entry &entry : drained)
        if(entry.text.find(text) != std::string::npos)
            return true;

    return false;
}

std::string summary_of(std::span<const capability_view> views)
{
    return std::to_string(praxis::count_defaults(views)) + " composed slots hold an inert default";
}

}

TEST_CASE("the summary counts the composed slots holding an inert default and arrives at the informational level", "[scene]")
{
    const sample_ops unbound;
    const std::array<capability_view, 1> views{capability_view::of(unbound, described_sample)};
    const std::vector<log_entry> drained = reported_at(severity::info, views);

    REQUIRE(entries_at(drained, severity::info) == 1);
    REQUIRE(mentions(drained, "2 composed slots hold an inert default"));
}

TEST_CASE("no per-slot message is emitted at the level an operator sees without changing a setting", "[scene]")
{
    const sample_ops unbound;
    const std::array<capability_view, 1> views{capability_view::of(unbound, described_sample)};
    const std::vector<log_entry> drained = reported_at(severity::info, views);

    REQUIRE(entries_at(drained, severity::debug) == 0);
    REQUIRE_FALSE(mentions(drained, "sample.first"));
}

TEST_CASE("the lowest level enumerates one message per defaulted slot, each naming that slot", "[scene]")
{
    const sample_ops unbound;
    const std::array<capability_view, 1> views{capability_view::of(unbound, described_sample)};
    const std::vector<log_entry> drained = reported_at(severity::debug, views);

    REQUIRE(entries_at(drained, severity::debug) == praxis::count_defaults(views));
    REQUIRE(mentions(drained, "sample.first"));
    REQUIRE(mentions(drained, "sample.second"));
}

TEST_CASE("the summary's count is the count the enumeration reports for the same views", "[scene]")
{
    SECTION("nothing bound")
    {
        const sample_ops unbound;
        const std::array<capability_view, 1> views{capability_view::of(unbound, described_sample)};

        REQUIRE(praxis::count_defaults(views) == 2);
        REQUIRE(mentions(reported_at(severity::info, views), summary_of(views)));
    }

    SECTION("one slot bound")
    {
        const sample_ops partly{.second = &bound_second};
        const std::array<capability_view, 1> views{capability_view::of(partly, described_sample)};
        const std::vector<log_entry> drained = reported_at(severity::debug, views);

        REQUIRE(praxis::count_defaults(views) == 1);
        REQUIRE(mentions(drained, summary_of(views)));
        REQUIRE(mentions(drained, "sample.first"));
        REQUIRE_FALSE(mentions(drained, "sample.second"));
    }
}
