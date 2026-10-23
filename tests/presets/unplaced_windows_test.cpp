#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/rotation_axis.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>

using namespace praxis;
using namespace praxis::fixture;

namespace {

void check_none_placed(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);
    REQUIRE_FALSE(composed->windows.empty());
    for(const std::shared_ptr<scene::imgui_window> &window : composed->windows)
    {
        INFO(window->display_name());
        CHECK_FALSE(window->opening().has_value());
    }
}

}

TEST_CASE("a window the rigid-motion demonstration composes carries no opening placement", "[presets][windows]")
{
    threepp::Scene target;

    check_none_placed(presets::rotation_axis_preset(unwired(target), rigid_motion::baseline()));
}

TEST_CASE("a window the manipulator demonstration composes carries no opening placement", "[presets][windows]")
{
    const described_arm described(1, "one_link");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    check_none_placed(presets::arm_preset(unwired(target), manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline(), optimization::baseline(), chosen,
                                          presets::arm_windows(chosen)));
}
