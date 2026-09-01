#ifndef HPP_GUARD_PRAXIS_SCENE_RATES_H
#define HPP_GUARD_PRAXIS_SCENE_RATES_H

#include "praxis/scheduler/task.h"

namespace praxis::scene {

// The period a preset's own sampled work is registered at when its author names none of its own. A
// preset whose sampling is cheaper or coarser than the frames it feeds is free to choose another.
inline constexpr scheduler::step_period sampled_preset_period{scheduler::seconds{0.004}};

}

#endif
