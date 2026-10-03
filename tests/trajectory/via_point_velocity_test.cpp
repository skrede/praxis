#include "captured_log.h"
#include "configuration_run.h"

#include "via_point_cubics.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double via_point_tolerance = 1.0e-9;
constexpr double bound_overshoot     = 1.0e-9;
constexpr std::size_t per_segment    = 1000u;

// Lynch & Park, Modern Robotics, sec. 9.3, Fig. 9.7(a) and 9.8: the via points and velocities the
// figure plots, at T = 0, 1, 2, 3.
via_point_run figure_run()
{
    const std::array<configuration, 4> via      = {pair_of(0.0, 0.0), pair_of(0.0, 1.0), pair_of(1.0, 1.0), pair_of(1.0, 0.0)};
    const std::array<configuration, 4> velocity = {pair_of(0.0, 0.0), pair_of(1.0, 0.0), pair_of(0.0, -1.0), pair_of(0.0, 0.0)};

    via_point_run run{{0.0, 1.0, 2.0, 3.0}, {}};
    for(std::size_t j = 0u; j < 3u; ++j)
    {
        segment_coefficient_block block(2, 4);
        for(Eigen::Index i = 0; i < 2; ++i)
            block.row(i) = segment_coefficients(via[j][i], via[j + 1u][i], velocity[j][i], velocity[j + 1u][i], 1.0).transpose();
        run.segments.push_back(block);
    }

    return run;
}

double fastest_on_segments(std::span<const configuration> via)
{
    const auto motion               = commanded(via, pair_of(-5.0, -5.0));
    const std::vector<double> times = knots(via, motion->duration());
    const configuration velocity    = bounds().velocity;

    double reached = 0.0;
    for(std::size_t k = 0u; k + 1u < times.size(); ++k)
        for(std::size_t step = 0u; step <= per_segment; ++step)
        {
            const double t = times[k] + (times[k + 1u] - times[k]) * static_cast<double>(step) / static_cast<double>(per_segment);
            for(Eigen::Index i = 0; i < velocity.size(); ++i)
                reached = std::max(reached, std::abs(at(*motion, t).velocity[i]) / velocity[i]);
        }

    return reached;
}

}

// Biagiotti & Melchiorri, sec. 2.1.4, Example 2.7, with the intermediate velocities of Example 2.8.
TEST_CASE("the_book_example_points_take_the_interior_velocities_eq_2_3_gives")
{
    const std::array<double, 5> t = {0.0, 2.0, 4.0, 8.0, 10.0};
    const std::array<double, 5> q = {10.0, 20.0, 0.0, 30.0, 40.0};
    std::array<double, 4> slope{};
    for(std::size_t k = 1u; k < t.size(); ++k)
        slope[k - 1u] = (q[k] - q[k - 1u]) / (t[k] - t[k - 1u]);

    CHECK(slope == std::array<double, 4>{5.0, -10.0, 7.5, 5.0});
    CHECK(via_velocity(slope[0], slope[1]) == 0.0);
    CHECK(via_velocity(slope[1], slope[2]) == 0.0);
    CHECK(via_velocity(slope[2], slope[3]) == 6.25);
}

TEST_CASE("a_zero_slope_has_a_sign_of_its_own_so_it_agrees_only_with_another_zero")
{
    CHECK(via_velocity(-1.0, 0.0) == 0.0);
    CHECK(via_velocity(0.0, 1.0) == 0.0);
    CHECK(via_velocity(1.0, +0.0) == 0.0);
    CHECK(via_velocity(0.0, 0.0) == 0.0);
    CHECK(via_velocity(+0.0, -0.0) == 0.0);
    CHECK(via_velocity(2.0, 4.0) == 3.0);
    CHECK(via_velocity(-2.0, -4.0) == -3.0);
}

// Eq. (9.26)-(9.29) solved by hand for each segment of the figure, joint x in the first row.
TEST_CASE("the_figure_via_points_give_the_coefficients_the_book_equations_predict")
{
    const via_point_run run = figure_run();

    CHECK(run.segments[0].row(0) == Eigen::RowVector4d(0.0, 0.0, -1.0, 1.0));
    CHECK(run.segments[0].row(1) == Eigen::RowVector4d(0.0, 0.0, 3.0, -2.0));
    CHECK(run.segments[1].row(0) == Eigen::RowVector4d(0.0, 1.0, 1.0, -1.0));
    CHECK(run.segments[1].row(1) == Eigen::RowVector4d(1.0, 0.0, 1.0, -1.0));
    CHECK(run.segments[2].row(0) == Eigen::RowVector4d(1.0, 0.0, 0.0, 0.0));
    CHECK(run.segments[2].row(1) == Eigen::RowVector4d(1.0, -1.0, -1.0, 1.0));
}

TEST_CASE("a_sample_at_an_interior_knot_belongs_to_the_later_segment_and_one_beyond_either_end_is_held_there")
{
    const auto motion = via_point_generator(figure_run());

    CHECK(motion->duration() == 3.0);
    CHECK(at(*motion, 1.0).acceleration == pair_of(2.0, 2.0));
    CHECK(at(*motion, 3.0).acceleration == pair_of(0.0, 4.0));
    CHECK(at(*motion, -1.0).position == at(*motion, 0.0).position);
    CHECK(at(*motion, -1.0).acceleration == at(*motion, 0.0).acceleration);
    CHECK(at(*motion, 4.0).position == pair_of(1.0, 0.0));
    CHECK(at(*motion, 4.0).acceleration == at(*motion, 3.0).acceleration);
}

TEST_CASE("a_run_of_via_points_passes_each_interior_via_point_at_the_velocity_eq_2_3_gives_for_its_own_knots")
{
    const std::vector<configuration> via = {pair_of(0.0, 0.0), pair_of(1.0, 0.5), pair_of(2.5, 0.75), pair_of(3.0, 1.5), pair_of(2.0, 2.0)};
    const auto motion                    = commanded(via, pair_of(-5.0, -5.0));
    const std::vector<double> times      = knots(via, motion->duration());

    for(std::size_t k = 1u; k + 1u < via.size(); ++k)
    {
        const configuration oracle = book_velocity(via, times, k);

        INFO("the interior via point: " << k);
        REQUIRE(!oracle.isZero());
        CHECK(is_approx_equal(at(*motion, times[k]).velocity, oracle, via_point_tolerance));
    }
}

TEST_CASE("a_run_holding_a_coordinate_that_is_not_finite_is_refused_and_the_row_and_degree_of_freedom_are_named")
{
    for(const double unfinished : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        const std::vector<configuration> via = {pair_of(1.0, 1.0), pair_of(2.0, unfinished), pair_of(3.0, 2.0)};

        praxis::tests::captured_log captured;
        const auto refused         = joint_space_waypoints(via, pair_of(-5.0, -5.0), bounds());
        const std::string reported = captured.text();

        INFO("the coordinate: " << unfinished);
        REQUIRE(!refused);
        CHECK(refused.error() == refusal::unsupported_input);
        CHECK(reported.find("row 1") != std::string::npos);
        CHECK(reported.find("degree of freedom 1") != std::string::npos);
    }
}

TEST_CASE("no_degree_of_freedom_of_a_run_of_via_points_crosses_its_velocity_bound_on_any_segment")
{
    const std::vector<configuration> fixed = {pair_of(0.0, 0.0), pair_of(1.0, 0.5), pair_of(0.5, 1.5), pair_of(2.0, 1.0)};
    CHECK(fastest_on_segments(fixed) <= 1.0 + bound_overshoot);

    std::mt19937 drawn(0x5EEDu);
    const auto coordinate = [&drawn] { return -2.0 + 4.0 * static_cast<double>(drawn()) / 4294967296.0; };
    for(std::size_t run = 0u; run < 20u; ++run)
    {
        std::vector<configuration> via;
        for(std::size_t row = 0u; row < 6u; ++row)
        {
            const double first = coordinate();
            via.push_back(pair_of(first, coordinate()));
        }

        INFO("the drawn run: " << run);
        CHECK(fastest_on_segments(via) <= 1.0 + bound_overshoot);
    }
}
