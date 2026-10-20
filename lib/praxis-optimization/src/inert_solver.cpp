#include "praxis/optimization/solver.h"

namespace praxis::optimization::inert {

expected<Eigen::VectorXd, refusal> solve(const problem &, const Eigen::VectorXd &, std::uint32_t, std::vector<Eigen::VectorXd> &)
{
    return unexpected(refusal::not_implemented);
}

}
