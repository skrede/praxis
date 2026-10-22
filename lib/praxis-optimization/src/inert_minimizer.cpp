#include "praxis/optimization/minimizer.h"

namespace praxis::optimization::inert {

expected<Eigen::VectorXd, refusal> newton(const problem &, const Eigen::VectorXd &, std::uint32_t, std::vector<Eigen::VectorXd> &)
{
    return unexpected(refusal::not_implemented);
}

expected<Eigen::VectorXd, refusal> gradient_descent(const problem &, const Eigen::VectorXd &, double, std::uint32_t, std::vector<Eigen::VectorXd> &)
{
    return unexpected(refusal::not_implemented);
}

}
