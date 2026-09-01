#include "praxis/manipulator/option_cycle.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <cstdint>

namespace {

enum class beacon : std::uint8_t
{
    alpha,
    bravo,
    charlie,
    delta
};

constexpr std::array<beacon, 4> beacon_options{beacon::alpha, beacon::bravo, beacon::charlie, beacon::delta};
constexpr std::array<const char *, 4> beacon_labels{"alpha", "bravo", "charlie", "delta"};

praxis::manipulator::option_cycle<beacon, 4> beacon_cycle(beacon initial = beacon::delta)
{
    return {initial, beacon_options, beacon_labels};
}

}

TEST_CASE("options are carried through in value, size and order")
{
    const auto cycle = beacon_cycle();

    const auto options = cycle.options();
    REQUIRE(options.size() == beacon_options.size());

    for(std::size_t i = 0; i < beacon_options.size(); ++i)
        REQUIRE(options[i] == beacon_options[i]);
}

TEST_CASE("labels are carried through in value, size and order")
{
    const auto cycle = beacon_cycle();

    const auto labels = cycle.labels();
    REQUIRE(labels.size() == beacon_labels.size());

    for(std::size_t i = 0; i < beacon_labels.size(); ++i)
        REQUIRE(std::string(labels[i]) == beacon_labels[i]);
}

TEST_CASE("every option maps to the label declared beside it")
{
    const auto cycle = beacon_cycle();

    for(std::size_t i = 0; i < beacon_options.size(); ++i)
        REQUIRE(cycle.label(beacon_options[i]) == beacon_labels[i]);
}

TEST_CASE("the initial option is the current value and names the current label")
{
    const auto cycle = beacon_cycle(beacon_options[2]);

    REQUIRE(cycle.value() == beacon_options[2]);
    REQUIRE(cycle.label() == beacon_labels[2]);
    REQUIRE(cycle.value_index() == 2);
}

TEST_CASE("setting an option updates the value and the label")
{
    auto cycle = beacon_cycle(beacon_options[2]);

    cycle.set(beacon_options[3]);

    REQUIRE(cycle.value() == beacon_options[3]);
    REQUIRE(cycle.label() == beacon_labels[3]);
}

TEST_CASE("setting a known label updates the value and reports success")
{
    auto cycle = beacon_cycle(beacon_options[2]);

    REQUIRE(cycle.set(beacon_labels[3]));
    REQUIRE(cycle.value() == beacon_options[3]);
    REQUIRE(cycle.label() == beacon_labels[3]);
}

TEST_CASE("an unknown label is reported as absent and leaves the value alone")
{
    auto cycle = beacon_cycle(beacon_options[2]);

    REQUIRE(cycle.option_of("echo") == std::nullopt);
    REQUIRE_FALSE(cycle.set("echo"));
    REQUIRE(cycle.value() == beacon_options[2]);
}

TEST_CASE("a known label resolves to the option declared beside it")
{
    const auto cycle = beacon_cycle();

    for(std::size_t i = 0; i < beacon_labels.size(); ++i)
        REQUIRE(cycle.option_of(beacon_labels[i]) == beacon_options[i]);
}

TEST_CASE("comparison against an option")
{
    const auto cycle = beacon_cycle(beacon_options[2]);

    REQUIRE(cycle == beacon_options[2]);
    REQUIRE(cycle != beacon_options[0]);
    REQUIRE(cycle != beacon_options[1]);
    REQUIRE(cycle != beacon_options[3]);
}

TEST_CASE("comparison against a label, including one the set does not carry")
{
    const auto cycle = beacon_cycle(beacon_options[2]);

    REQUIRE(cycle == beacon_labels[2]);
    REQUIRE(cycle != beacon_labels[0]);
    REQUIRE(cycle != beacon_labels[1]);
    REQUIRE(cycle != beacon_labels[3]);
    REQUIRE(cycle != "echo");
}

TEST_CASE("an option outside the set has no index and no label")
{
    auto cycle = beacon_cycle(beacon_options[0]);

    cycle.set(static_cast<beacon>(9));

    REQUIRE(cycle.value_index() == std::nullopt);
    REQUIRE(cycle.label().empty());
}
