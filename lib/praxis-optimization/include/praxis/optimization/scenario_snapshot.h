#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_SNAPSHOT_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_SNAPSHOT_H

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/scheduler/snapshot.h"

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace praxis::optimization {

struct interval
{
    double lower;
    double upper;
};

// One solve from one seed. The error and the Jacobian are the problem's own answers at each
// iterate, index for index, so a reader draws a step without calling a registered function. `answer`
// is what the solve answered, and `refused` its refusal, or no_solution where the problem's own
// converged test does not hold at the answer. A run in the minimum form names the minimizer it was
// solved by and carries f, its gradient and, where the problem carries one, its Hessian at each
// iterate as well, judging the answer by the problem's stationary test instead; the root form
// leaves the four empty.
struct scenario_run
{
    Eigen::VectorXd seed;
    std::uint32_t limit;
    std::vector<Eigen::VectorXd> iterates;
    std::vector<expected<Eigen::VectorXd, refusal>> errors;
    std::vector<expected<Eigen::MatrixXd, refusal>> jacobians;
    std::optional<refusal> refused;
    std::optional<Eigen::VectorXd> answer;
    std::vector<expected<double, refusal>> objectives;
    std::vector<expected<Eigen::VectorXd, refusal>> gradients;
    std::vector<expected<Eigen::MatrixXd, refusal>> hessians;
    std::optional<minimizer_slot> minimized_by;
};

// A Jacobian's central-difference estimate and the largest relative difference of their entries,
// |J - J_cd| / max(1, |J|, |J_cd|), which disagrees above the check's tolerance.
struct derivative_check
{
    Eigen::MatrixXd central;
    double difference;
    bool disagrees;
};

// The problem's own answers at one point; the check is present only where the composition opted
// into it and every value it needed was answered.
struct scenario_linearization
{
    Eigen::VectorXd point;
    expected<Eigen::VectorXd, refusal> error;
    expected<Eigen::MatrixXd, refusal> jacobian;
    std::optional<derivative_check> check;
};

// A function of two variables sampled at `count` evenly spaced points along each range, ends
// included, x varying fastest: f(x_i, y_j) is heights[j count + i], and NaN where f was not answered
// finitely.
struct scenario_grid
{
    std::size_t count;
    std::vector<double> heights;
};

// A function of one variable is sampled evenly over its range, ends included, into the abscissa and
// the ordinate, and one of two variables into the grid, which publications that solve nothing again
// share; a sample the function did not answer finitely is NaN, which breaks the drawing there.
// `step` is the step k selected, from x_k to x_(k+1), and `stepped` says the run was stopped by
// stepping once short of the iteration limit, so a no_solution it carries without an answer says
// only that its steps ran out. `bounds` is the box the run was solved in, and `linearization` is
// present once a point has been asked for.
struct scenario_snapshot
{
    bool stepped;
    std::uint32_t step;
    std::size_t selected;
    std::vector<interval> ranges;
    std::vector<double> abscissa;
    std::vector<double> ordinate;
    std::shared_ptr<const scenario_grid> grid;
    scenario_run run;
    std::optional<box> bounds;
    std::optional<scenario_linearization> linearization;
};

using scenario_publisher = scheduler::snapshot_publisher<std::shared_ptr<const scenario_snapshot>>;
using scenario_reader    = scheduler::snapshot_reader<std::shared_ptr<const scenario_snapshot>>;

}

#endif
