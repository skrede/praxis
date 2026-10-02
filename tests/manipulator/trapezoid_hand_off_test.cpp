#include "fixtures.h"

#include "book_time_scalings.h"

#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// The preview samples a motion at span * i / 384 for i from 0 to this.
constexpr std::size_t last_instant = 384;

const joint_vector from = configuration(0.1, -0.2);

joint_limits two_joint_limits()
{
    joint_limits limits{};
    limits.velocity     = joint_vector::Constant(2, 0.5);
    limits.acceleration = joint_vector::Constant(2, 2.0);

    return limits;
}

struct bounded_motion
{
    path_parameter_bounds held;
    joint_vector target;
};

std::vector<bounded_motion> population()
{
    std::vector<bounded_motion> pairs;
    for(int k = 0; k < 64; ++k)
    {
        const double reach         = 1.0e-4 * std::pow(3.0e4, static_cast<double>(k) / 63.0);
        const joint_vector reached = joint_vector(from + configuration(reach, 0.5 * reach));
        pairs.push_back({derived_bounds(two_joint_limits(), from, reached), reached});
    }

    const joint_vector to = configuration(0.9, 0.6);
    for(const double rate_change : {0.05, 0.5, 5.0, 50.0})
        pairs.push_back({{std::sqrt(rate_change), rate_change}, to});
    for(int i = 0; i < 32; ++i)
        for(int j = 0; j < 32; ++j)
            pairs.push_back({{0.4 + 2.6 * static_cast<double>(i) / 31.0, 0.5 + 9.5 * static_cast<double>(j) / 31.0}, to});

    return pairs;
}

// A form whose check never computes a natural duration runs the same arithmetic whichever natural
// duration it is labeled with.
bool distinct(const trapezoid_form &form)
{
    const trapezoid_form first = trapezoid_forms.front();

    return form.check == duration_check::natural || (form.apex_when == first.apex_when && form.coast == first.coast && form.apex == first.apex);
}

bool sound(const expected<trajectory::trajectory_sample, refusal> &sampled)
{
    return sampled && sampled->position.allFinite() && sampled->velocity.allFinite() && sampled->acceleration.allFinite();
}

struct failure
{
    std::size_t form;
    std::size_t pair;
    std::size_t instant;
};

std::size_t failures_over(trapezoid_slot literal, failure at, const bounded_motion &bounded, std::optional<failure> &first)
{
    trajectory::time_scaling_ops ops = trajectory::baseline().time_scaling;
    ops.trapezoidal                  = literal;

    const prepared_time_scaling scaled(ops, time_scaling_choice::trapezoidal, bounded.held);
    const auto motion = joint_space_motion(composing_path(), scaled, two_joint_limits(), from, bounded.target);
    if(!motion)
        return last_instant + 1;

    std::size_t failed = 0;
    const double span  = (*motion)->duration();
    for(std::size_t i = 0; i <= last_instant; ++i)
    {
        if(sound((*motion)->sample(span * static_cast<double>(i) / static_cast<double>(last_instant))))
            continue;
        if(!first)
            first = failure{at.form, at.pair, i};
        ++failed;
    }

    return failed;
}

std::vector<double> asked;

expected<trajectory::scaling_sample, refusal> recorded_cubic(double t, double duration)
{
    asked.push_back(t);

    return trajectory::baseline().time_scaling.cubic(t, duration);
}

expected<trajectory::scaling_sample, refusal> recorded_quintic(double t, double duration)
{
    asked.push_back(t);

    return trajectory::baseline().time_scaling.quintic(t, duration);
}

expected<trajectory::scaling_sample, refusal> recorded_trapezoidal(double t, double duration, double max_velocity, double max_acceleration)
{
    asked.push_back(t);

    return trajectory::baseline().time_scaling.trapezoidal(t, duration, max_velocity, max_acceleration);
}

trajectory::time_scaling_ops recording_time_scaling()
{
    return trajectory::time_scaling_ops{.cubic = &recorded_cubic, .quintic = &recorded_quintic, .trapezoidal = &recorded_trapezoidal};
}

}

TEST_CASE("every_book_trapezoid_runs_each_motion_the_scene_composes_to_its_end")
{
    const std::vector<bounded_motion> pairs = population();
    const auto literals                     = book_trapezoids();

    std::size_t failed = 0;
    std::optional<failure> first;
    for(std::size_t form = 0; form < trapezoid_form_count; ++form)
        for(std::size_t pair = 0; distinct(trapezoid_forms[form]) && pair < pairs.size(); ++pair)
            failed += failures_over(literals[form], failure{form, pair, 0}, pairs[pair], first);

    if(first)
        UNSCOPED_INFO("first failure: form " << first->form << " rate " << pairs[first->pair].held.max_rate << " rate change " << pairs[first->pair].held.max_rate_change << " instant "
                                             << first->instant);
    CHECK(failed == 0u);
}

TEST_CASE("no_scaling_is_asked_about_a_time_outside_the_duration_it_is_handed")
{
    constexpr double duration = 2.7;
    const std::array<std::pair<double, double>, 4> asks{
            {{-1.0, 0.0}, {duration * (1.0 + 0x1p-52), duration}, {std::nextafter(duration, std::numeric_limits<double>::infinity()), duration}, {2.0 * duration, duration}}};
    for(const time_scaling_choice chosen : {time_scaling_choice::cubic, time_scaling_choice::quintic, time_scaling_choice::trapezoidal})
    {
        const prepared_time_scaling scaled(recording_time_scaling(), chosen, path_parameter_bounds{1.0, 4.0});
        for(const auto &[t, held_at] : asks)
        {
            INFO("choice " << static_cast<int>(chosen) << " asked at " << t);
            asked.clear();
            CHECK(scaled.sample(t, duration).has_value());
            REQUIRE(asked.size() == 1u);
            CHECK(asked.front() == held_at);
        }
    }
}
