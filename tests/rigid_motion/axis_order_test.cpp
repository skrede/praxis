#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <string>
#include <cstddef>
#include <numbers>
#include <string_view>

using namespace praxis;

namespace {

constexpr std::array<axis_order, 12> orders{axis_order::xyz, axis_order::xzy, axis_order::yxz, axis_order::yzx, axis_order::zxy, axis_order::zyx,
                                            axis_order::xyx, axis_order::xzx, axis_order::yxy, axis_order::yzy, axis_order::zxz, axis_order::zyz};

constexpr std::array<std::string_view, 12> spellings{"xyz", "xzy", "yxz", "yzx", "zxy", "zyx", "xyx", "xzx", "yxy", "yzy", "zxz", "zyz"};

std::string upper(std::string_view lower)
{
    std::string result;
    for(char letter : lower)
        result.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(letter))));
    return result;
}

constexpr double round_trip_tolerance = 4.0 * std::numeric_limits<double>::epsilon();

void require_round_trip(double value, double returned)
{
    REQUIRE(std::abs(returned - value) <= std::abs(value) * round_trip_tolerance);
}

std::uint8_t expected_index(char axis)
{
    if(axis == 'x')
        return std::uint8_t{0};
    if(axis == 'y')
        return std::uint8_t{1};
    return std::uint8_t{2};
}

}

TEST_CASE("every_order_has_a_label_spelling_its_own_axes")
{
    REQUIRE(axis_order_labels().size() == orders.size());

    for(std::size_t i = 0; i < orders.size(); ++i)
    {
        const std::size_t index = static_cast<std::size_t>(orders[i]);
        REQUIRE(index == i);
        REQUIRE(std::string(axis_order_labels()[index]) == upper(spellings[i]));
    }
}

TEST_CASE("axis_indices_names_the_axes_in_the_order_the_enumerator_spells_them")
{
    for(std::size_t i = 0; i < orders.size(); ++i)
    {
        const std::array<std::uint8_t, 3> indices = axis_indices(orders[i]);
        for(std::size_t axis = 0; axis < indices.size(); ++axis)
            REQUIRE(indices[axis] == expected_index(spellings[i][axis]));
    }
}

TEST_CASE("the_proper_euler_orders_repeat_their_first_axis")
{
    REQUIRE(axis_indices(axis_order::xyx) == std::array<std::uint8_t, 3>{0, 1, 0});
    REQUIRE(axis_indices(axis_order::xzx) == std::array<std::uint8_t, 3>{0, 2, 0});
    REQUIRE(axis_indices(axis_order::yxy) == std::array<std::uint8_t, 3>{1, 0, 1});
    REQUIRE(axis_indices(axis_order::yzy) == std::array<std::uint8_t, 3>{1, 2, 1});
    REQUIRE(axis_indices(axis_order::zxz) == std::array<std::uint8_t, 3>{2, 0, 2});
    REQUIRE(axis_indices(axis_order::zyz) == std::array<std::uint8_t, 3>{2, 1, 2});
}

TEST_CASE("an_angle_returns_to_itself_through_both_conversions")
{
    constexpr std::array<double, 4> entered_degrees{0.0, 360.0, -45.0, 30.5};
    constexpr std::array<double, 4> entered_radians{0.0, 2.0 * std::numbers::pi, -std::numbers::pi / 4.0, 0.5321};

    for(double value : entered_degrees)
        require_round_trip(value, to_degrees(to_radians(value)));

    for(double value : entered_radians)
        require_round_trip(value, to_radians(to_degrees(value)));
}
