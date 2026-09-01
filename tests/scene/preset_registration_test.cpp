#include "praxis/scene/preset_registry.h"

#include <catch2/catch_test_macros.hpp>

#include <type_traits>

using praxis::scene::preset_registry;

TEST_CASE("every registry operation is reachable only through an instance", "[scene]")
{
    STATIC_REQUIRE(std::is_member_function_pointer_v<decltype(&preset_registry::register_preset)>);
    STATIC_REQUIRE(std::is_member_function_pointer_v<decltype(&preset_registry::load_preset)>);
    STATIC_REQUIRE(std::is_member_function_pointer_v<decltype(&preset_registry::preset_names)>);
}

TEST_CASE("a freshly constructed registry answers an empty name list", "[scene]")
{
    const preset_registry fresh;

    REQUIRE(fresh.preset_names().empty());
}
