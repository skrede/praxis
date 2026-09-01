#include "praxis/extension.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;

namespace {

double zero_offset()
{
    return 0.0;
}

double unit_scale()
{
    return 1.0;
}

double measured_scale()
{
    return 2.0;
}

double zero_bias()
{
    return 0.0;
}

std::size_t no_samples()
{
    return 0;
}

struct meter_ops
{
    double (*offset)() = &zero_offset;
    double (*scale)()  = &unit_scale;
};

struct gauge_ops
{
    double (*bias)()         = &zero_bias;
    std::size_t (*samples)() = &no_samples;
};

enum class meter_slot : std::uint32_t
{
    offset,
    scale,
    count
};

enum class gauge_slot : std::uint32_t
{
    bias,
    samples,
    count
};

constexpr std::array meter_descriptors{
        slot_descriptor{"meter.offset", [](const void *value) -> bool { return static_cast<const meter_ops *>(value)->offset == &zero_offset; }},
        slot_descriptor{"meter.scale", [](const void *value) -> bool { return static_cast<const meter_ops *>(value)->scale == &unit_scale; }},
};

constexpr std::array gauge_descriptors{
        slot_descriptor{"gauge.bias", [](const void *value) -> bool { return static_cast<const gauge_ops *>(value)->bias == &zero_bias; }},
        slot_descriptor{"gauge.samples", [](const void *value) -> bool { return static_cast<const gauge_ops *>(value)->samples == &no_samples; }},
};

static_assert(meter_descriptors.size() == static_cast<std::size_t>(meter_slot::count));
static_assert(gauge_descriptors.size() == static_cast<std::size_t>(gauge_slot::count));

constexpr capability_descriptors<meter_ops> described_meters{"instrument", meter_descriptors};
constexpr capability_descriptors<gauge_ops> described_gauges{"telemetry", gauge_descriptors};

capability_view view_of(const meter_ops &ops)
{
    return capability_view::of(ops, described_meters);
}

capability_view view_of(const gauge_ops &ops)
{
    return capability_view::of(ops, described_gauges);
}

}

TEST_CASE("each_descriptor_table_covers_every_slot_once_under_a_unique_non_empty_name")
{
    meter_ops meter{};
    gauge_ops gauge{};
    const std::array<capability_view, 2> views{view_of(meter), view_of(gauge)};
    std::set<std::string_view> names;

    for(const capability_view &view : views)
    {
        for(const slot_descriptor &descriptor : view.slots())
        {
            REQUIRE_FALSE(descriptor.name.empty());
            REQUIRE(names.insert(descriptor.name).second);
        }
    }
    REQUIRE(names.size() == meter_descriptors.size() + gauge_descriptors.size());
}

TEST_CASE("a_value_initialized_capability_reports_every_slot_as_holding_its_default")
{
    meter_ops meter{};
    const std::array<capability_view, 1> views{view_of(meter)};

    REQUIRE(count_defaults(views) == meter_descriptors.size());

    for(std::size_t index = 0; index < meter_descriptors.size(); ++index)
    {
        REQUIRE(holds_default(views.front(), index));
    }
}

TEST_CASE("overriding_one_member_leaves_every_other_slot_reported_as_holding_its_default")
{
    meter_ops meter{.scale = &measured_scale};
    const capability_view view = view_of(meter);
    const std::array<capability_view, 1> views{view};

    REQUIRE(count_defaults(views) == meter_descriptors.size() - 1u);
    REQUIRE_FALSE(holds_default(view, static_cast<std::size_t>(meter_slot::scale)));
    REQUIRE(holds_default(view, static_cast<std::size_t>(meter_slot::offset)));
    REQUIRE(slot_name(view, static_cast<std::size_t>(meter_slot::scale)) == "meter.scale");
}

TEST_CASE("the_report_names_slots_from_both_capabilities_under_their_own_extension_labels")
{
    meter_ops meter{.scale = &measured_scale};
    gauge_ops gauge{};
    const std::array<capability_view, 2> views{view_of(meter), view_of(gauge)};
    const std::vector<defaulted_slot> report = defaulted_slots(views);
    std::set<std::string> qualified;

    REQUIRE(report.size() == meter_descriptors.size() + gauge_descriptors.size() - 1u);
    REQUIRE(count_defaults(views) == report.size());

    for(const defaulted_slot &entry : report)
    {
        REQUIRE_FALSE(entry.slot.empty());
        REQUIRE(qualified.insert(std::string(entry.extension).append(".").append(entry.slot)).second);
    }
    REQUIRE(qualified.count("instrument.meter.offset") == 1u);
    REQUIRE(qualified.count("telemetry.gauge.samples") == 1u);
}

TEST_CASE("an_index_past_the_last_slot_answers_false_and_the_empty_name")
{
    meter_ops meter{};
    const capability_view view = view_of(meter);
    const std::size_t past     = static_cast<std::size_t>(meter_slot::count);

    REQUIRE_FALSE(holds_default(view, past));
    REQUIRE(slot_name(view, past).empty());
    REQUIRE(slot_name(view, 0) == "meter.offset");
}

TEST_CASE("a_default_constructed_view_answers_false_and_the_empty_name_at_every_index")
{
    const capability_view view;
    const std::array<capability_view, 1> views{view};

    REQUIRE(view.extension().empty());
    REQUIRE(view.slots().empty());
    REQUIRE(view.value() == nullptr);
    REQUIRE(count_defaults(views) == 0u);
    REQUIRE(defaulted_slots(views).empty());

    for(std::size_t index : {std::size_t{0}, std::size_t{1}, meter_descriptors.size(), std::size_t{4096}})
    {
        REQUIRE_FALSE(holds_default(view, index));
        REQUIRE(slot_name(view, index).empty());
    }
}
