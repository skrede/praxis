#include "evaluation_driving.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/types.h"

#include <cmath>
#include <cstddef>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// How many times one comparison samples the two generators at. One count serves every case, so what
// a case costs does not follow the duration a factory answers.
constexpr std::size_t samples_per_case = 8u;

// Seconds at the floor of the bound two spans agree within, and a relative bound above it.
constexpr double duration_agreement_seconds = 1.0e-12;

// What the times one case sampled have said so far. `parted` holds the first outcome at which the
// two sides did not say the same thing about a time, and `not_exercised` stands for no such time yet.
struct run_tally
{
    evaluation::residual worst;
    std::size_t answered;
    evaluation::agreement parted;
};

// A sample of another width stands in another configuration space, so no element of one corresponds
// to an element of the other.
bool widths_agree(const trajectory_sample &held, const trajectory_sample &against)
{
    return held.position.size() == against.position.size() && held.velocity.size() == against.velocity.size() && held.acceleration.size() == against.acceleration.size();
}

double worst_element(const configuration &held, const configuration &against)
{
    return evaluation::element_wise_residual(held, against).magnitude;
}

// Three configurations, in radians and in radians per second and per second squared. The residual
// carries one number for the three, so they fold to whichever of them is loosest.
evaluation::residual folded(const trajectory_sample &held, const trajectory_sample &against)
{
    if(!widths_agree(held, against))
        return differing(evaluation::residual_kind::element_wise).difference;

    const double worst =
            std::max({worst_element(held.position, against.position), worst_element(held.velocity, against.velocity), worst_element(held.acceleration, against.acceleration)});

    return evaluation::residual{evaluation::residual_kind::element_wise, worst, 0.0};
}

// The two units stay apart. Every angle among them -- the transform's geodesic distance and the two
// twists' angular parts -- goes in the magnitude half in radians, and every length -- the transform's
// origin distance and the two twists' linear parts -- goes in the linear half in metres. Neither half
// is ever added to the other.
evaluation::residual folded(const pose_sample &held, const pose_sample &against)
{
    const evaluation::residual placed = evaluation::pose_residual(held.position, against.position);
    const twist moving                = held.velocity - against.velocity;
    const twist speeding              = held.acceleration - against.acceleration;

    return evaluation::residual{evaluation::residual_kind::pose, std::max({placed.magnitude, moving.head<3>().norm(), speeding.head<3>().norm()}),
                                std::max({placed.linear_error_metres, moving.tail<3>().norm(), speeding.tail<3>().norm()})};
}

bool durations_agree(double held, double against)
{
    return std::abs(held - against) <= duration_agreement_seconds * std::max({1.0, std::abs(held), std::abs(against)});
}

void record(run_tally &run, const evaluation::case_result &at_time)
{
    if(at_time.verdict == evaluation::agreement::one_refused || at_time.verdict == evaluation::agreement::refused_differently)
    {
        if(run.parted == evaluation::agreement::not_exercised)
            run.parted = at_time.verdict;

        return;
    }
    if(at_time.verdict == evaluation::agreement::both_refused)
        return;

    ++run.answered;
    run.worst.magnitude           = std::max(run.worst.magnitude, at_time.difference.magnitude);
    run.worst.linear_error_metres = std::max(run.worst.linear_error_metres, at_time.difference.linear_error_metres);
}

// A time the two sides parted on decides the case: a run is agreement only where every time it
// sampled either agreed or was declined alike by both sides. A run no time of which either side
// answered is agreement about the input and carries no number.
evaluation::case_result verdict_over(const run_tally &run, const evaluation::tolerance_pair &allowed)
{
    if(run.parted != evaluation::agreement::not_exercised)
        return evaluation::case_result{run.parted, evaluation::residual{}};
    if(run.answered == 0u)
        return evaluation::case_result{evaluation::agreement::both_refused, evaluation::residual{}};

    return judged(run.worst, allowed);
}

// One case's worth of driving, written once for the two prepared shapes this extension publishes.
// The refusal channel of a sample is carried by the facility's own policy, so a time the shorter
// span has run past is an outcome and not a number.
template<typename Generator, typename Sample>
evaluation::case_result over_the_shared_span(const Generator &held, const Generator &against, evaluation::residual_kind kind, const evaluation::tolerance_pair &allowed)
{
    const double span = held.duration();
    if(!durations_agree(span, against.duration()))
        return differing(kind);

    run_tally run{evaluation::residual{kind, 0.0, 0.0}, 0u, evaluation::agreement::not_exercised};

    for(std::size_t index = 0; index < samples_per_case; ++index)
    {
        const double at = span * static_cast<double>(index) / static_cast<double>(samples_per_case - 1u);

        record(run, evaluation::agreed_or_refused(held.sample(at), against.sample(at), [](const Sample &first, const Sample &second) { return folded(first, second); }, allowed));
    }

    return verdict_over(run, allowed);
}

}

evaluation::case_result driven(const trajectory_generator &held, const trajectory_generator &against, evaluation::residual_kind kind, const evaluation::tolerance_pair &allowed)
{
    return over_the_shared_span<trajectory_generator, trajectory_sample>(held, against, kind, allowed);
}

evaluation::case_result driven(const pose_trajectory_generator &held, const pose_trajectory_generator &against, evaluation::residual_kind kind,
                               const evaluation::tolerance_pair &allowed)
{
    return over_the_shared_span<pose_trajectory_generator, pose_sample>(held, against, kind, allowed);
}

}
