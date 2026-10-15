#include "composed_motion.h"
#include "bound_trajectories.h"

#include "../manipulator/substituted_trajectory.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// A stub bound to one slot over the reference, and the count showing it was the one run.
struct stubbed_slot
{
    const char *named;
    trajectory::capabilities (*bound)();
    std::size_t &(*asked)();
};

constexpr std::array via_point_stubs{stubbed_slot{"the step", &only_the_via_point_schedule, &via_point_schedule_asks},
                                     stubbed_slot{"the via-point slot", &only_the_via_point_slot, &via_point_slot_asks}};

}

TEST_CASE("a via-point scenario runs the step and the via-point slot its composition binds and draws what the reference draws", "[presets][seam][routing]")
{
    for(const stubbed_slot &one : via_point_stubs)
    {
        INFO(one.named);
        reset_counts();

        composed_motion reference(trajectory::baseline(), via_point);
        const drawn_lines drawn = reference.opening();

        REQUIRE(drawn_whole(drawn));
        REQUIRE(one.asked() == 0u);

        composed_motion stubbed(one.bound(), via_point);

        CHECK(furthest_apart(stubbed.opening(), drawn) <= 1.0e-12);
        CHECK(one.asked() > 0u);
        CHECK_FALSE(stubbed.unloaded);
    }
}

TEST_CASE("every trajectory scenario composed over each slot bound to the reference draws what the reference draws", "[presets][seam][routing]")
{
    for(const trajectory_composer &one : trajectory_composers)
    {
        INFO(one.named);

        composed_motion reference(trajectory::baseline(), one);
        const drawn_lines drawn = reference.opening();

        REQUIRE(drawn_whole(drawn));

        composed_motion bound(every_slot_the_reference(), one);

        CHECK(furthest_apart(bound.opening(), drawn) <= 1.0e-12);
        if(one.asked)
            CHECK(furthest_apart(bound.previewed(), reference.previewed()) <= 1.0e-12);
        CHECK_FALSE(bound.unloaded);
    }
}

TEST_CASE("the book's trapezoids and via-point join bound as transcribed open every trajectory scenario on the reference's motion", "[presets][seam][routing]")
{
    for(const trajectory_composer &one : trajectory_composers)
    {
        INFO(one.named);

        composed_motion reference(every_slot_the_reference(), one);
        const drawn_lines drawn = reference.opening();

        REQUIRE(drawn_whole(drawn));

        composed_motion literal(literal_book_slots(), one);
        const drawn_lines literal_drawn = literal.opening();

        CHECK(furthest_apart(literal_drawn, drawn) <= 1.0e-8);
        if(one.asked)
            CHECK(furthest_apart(short_of_the_rest(literal.previewed()), short_of_the_rest(reference.previewed())) <= 1.0e-8);
        CHECK_FALSE(literal.unloaded);
    }
}

TEST_CASE("a trapezoidal run chosen in the point-to-point scenario runs the velocity and acceleration trapezoid its composition binds", "[presets][seam][routing]")
{
    reset_counts();

    composed_motion reference(trajectory::baseline(), point_to_point);
    reference.choose_the_trapezoid();
    const drawn_lines drawn = reference.opening();

    REQUIRE(drawn_whole(drawn));
    REQUIRE(reference.chosen() == "Trapezoidal (chosen)");

    composed_motion stubbed(only_the_velocity_acceleration_trapezoid(), point_to_point);
    stubbed.choose_the_trapezoid();

    CHECK(furthest_apart(stubbed.opening(), drawn) <= 1.0e-12);
    CHECK(velocity_acceleration_asks() > 0u);
    CHECK_FALSE(stubbed.unloaded);

    composed_motion literal(literal_book_slots(), point_to_point);
    literal.choose_the_trapezoid();

    CHECK(furthest_apart(literal.opening(), drawn) <= 1.0e-8);
    CHECK(furthest_apart(literal.previewed(), reference.previewed()) <= 1.0e-8);
}

// The trapezoid drawn beside a quintic run spans the quintic's duration, which the acceleration and
// duration trapezoid is asked for.
TEST_CASE("the point-to-point scenario's opening preview runs the acceleration and duration trapezoid its composition binds", "[presets][seam][routing]")
{
    reset_counts();

    composed_motion reference(trajectory::baseline(), point_to_point);
    static_cast<void>(reference.opening());

    REQUIRE(reference.chosen() == "Quintic (chosen)");

    composed_motion stubbed(only_the_acceleration_duration_trapezoid(), point_to_point);
    static_cast<void>(stubbed.opening());

    CHECK(acceleration_duration_asks() > 0u);
    CHECK(furthest_apart(stubbed.previewed(), reference.previewed()) <= 1.0e-12);
    CHECK_FALSE(stubbed.unloaded);
}
