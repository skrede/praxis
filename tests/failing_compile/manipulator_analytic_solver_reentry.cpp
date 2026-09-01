#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

expected<void, refusal> analytic_solver_reentry(const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired, ik_result &answer)
{
    return forward.analytic_inverse_kinematics(forward, chain, desired, answer);
}

}
