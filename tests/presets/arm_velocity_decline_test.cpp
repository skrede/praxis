#include "captured_log.h"
#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <array>
#include <memory>
#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

struct dependency
{
    manipulator::differential_kinematics_slot slot;
    const char *named;
};

const std::array<dependency, 2> jacobians{dependency{manipulator::differential_kinematics_slot::space_jacobian, "dk.space_jacobian"},
                                          dependency{manipulator::differential_kinematics_slot::body_jacobian, "dk.body_jacobian"}};

manipulator::capabilities without(manipulator::capabilities composed, manipulator::differential_kinematics_slot slot)
{
    if(slot == manipulator::differential_kinematics_slot::space_jacobian)
        composed.dk.space_jacobian = &manipulator::inert::space_jacobian;
    if(slot == manipulator::differential_kinematics_slot::body_jacobian)
        composed.dk.body_jacobian = &manipulator::inert::body_jacobian;

    return composed;
}

// The sink stamps every line with the wall clock, so two reports are compared on what the
// composition said rather than on when it said it.
std::string said(const std::string &reported)
{
    const std::size_t begins = reported.find("praxis:");

    return begins == std::string::npos ? reported : reported.substr(begins);
}

std::shared_ptr<scene::preset> open_with(threepp::Scene &target, const presets::arm_scenario &chosen, const manipulator::capabilities &arm)
{
    return presets::arm_preset(unwired(target), arm, trajectory::baseline(), rigid_motion::baseline(), chosen, presets::arm_windows_velocity_kinematics(chosen));
}

}

TEST_CASE("a Jacobian left at its default composes no window and is named", "[presets][windows]")
{
    const described_arm described(6, "velocity_one_denied");
    const presets::arm_scenario chosen = described_by(described.where);

    for(const dependency &denied : jacobians)
    {
        INFO(denied.named);

        threepp::Scene target;
        std::string reported;
        std::shared_ptr<scene::preset> composed;
        {
            const tests::captured_log captured;
            composed = open_with(target, chosen, without(manipulator::baseline(), denied.slot));
            reported = captured.text();
        }

        REQUIRE(composed != nullptr);
        REQUIRE(composed->windows.empty());
        REQUIRE(reported.find(denied.named) != std::string::npos);

        for(const dependency &bound : jacobians)
            if(bound.slot != denied.slot)
            {
                INFO(bound.named);
                REQUIRE(reported.find(bound.named) == std::string::npos);
            }
    }
}

TEST_CASE("a decline names both Jacobians, in the order the enumeration declares them", "[presets][windows]")
{
    const described_arm described(6, "velocity_none_bound");
    const presets::arm_scenario chosen = described_by(described.where);

    manipulator::capabilities neither = manipulator::baseline();
    for(const dependency &denied : jacobians)
        neither = without(neither, denied.slot);

    threepp::Scene target;
    std::string reported;
    std::shared_ptr<scene::preset> composed;
    {
        const tests::captured_log captured;
        composed = open_with(target, chosen, neither);
        reported = captured.text();
    }

    REQUIRE(composed != nullptr);
    REQUIRE(composed->windows.empty());
    REQUIRE(reported.find("dk.space_jacobian") != std::string::npos);
    REQUIRE(reported.find("dk.body_jacobian") != std::string::npos);
    REQUIRE(reported.find("dk.space_jacobian") < reported.find("dk.body_jacobian"));
}

// A slot holding praxis's own inert function and a slot nobody touched are one state, because what
// separates a bound slot from an unbound one is which function it points at.
TEST_CASE("a Jacobian bound to praxis's own inert implementation reads as one left alone", "[presets][windows]")
{
    const described_arm described(6, "velocity_explicit_inert");
    const presets::arm_scenario chosen = described_by(described.where);

    manipulator::capabilities alone = manipulator::baseline();
    alone.dk                        = manipulator::differential_kinematics_ops{};

    manipulator::capabilities spelled = manipulator::baseline();
    spelled.dk.space_jacobian         = &manipulator::inert::space_jacobian;
    spelled.dk.body_jacobian          = &manipulator::inert::body_jacobian;

    threepp::Scene left;
    std::string by_default;
    std::shared_ptr<scene::preset> defaulted;
    {
        const tests::captured_log captured;
        defaulted  = open_with(left, chosen, alone);
        by_default = captured.text();
    }

    threepp::Scene right;
    std::string by_assignment;
    std::shared_ptr<scene::preset> assigned;
    {
        const tests::captured_log captured;
        assigned      = open_with(right, chosen, spelled);
        by_assignment = captured.text();
    }

    REQUIRE(defaulted != nullptr);
    REQUIRE(assigned != nullptr);
    REQUIRE(defaulted->windows.empty());
    REQUIRE(assigned->windows.empty());
    REQUIRE(said(by_default) == said(by_assignment));
}

TEST_CASE("both Jacobians bound composes windows and reports nothing", "[presets][windows]")
{
    const described_arm described(6, "velocity_both_bound");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    std::string reported;
    std::shared_ptr<scene::preset> composed;
    {
        const tests::captured_log captured;
        composed = open_with(target, chosen, manipulator::baseline());
        reported = captured.text();
    }

    REQUIRE(composed != nullptr);
    REQUIRE(!composed->windows.empty());
    REQUIRE(reported.empty());
}
