#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PROJECTED_STEP_CONSTANTS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PROJECTED_STEP_CONSTANTS_H

namespace praxis::manipulator::projected {

// The weights of the angular and the linear half of V_b in W_E, and the bias added to E on the diagonal.
inline constexpr double rotation_weight    = 1.0;
inline constexpr double translation_weight = 1.0;
inline constexpr double bias               = 1.0e-8;

}

#endif
