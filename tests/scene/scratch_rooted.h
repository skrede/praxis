#ifndef HPP_GUARD_PRAXIS_TESTS_SCENE_SCRATCH_ROOTED_H
#define HPP_GUARD_PRAXIS_TESTS_SCENE_SCRATCH_ROOTED_H

#include "../presets/scratch_directory.h"

#include "praxis/scene/visualizer.h"

namespace praxis::fixture {

// The options a visualizer opens at, rooted in the run's scratch directory so the layout the GUI
// library keeps is written there rather than into the directory the suite runs in.
inline scene::visualizer::options scratch_rooted()
{
    scene::visualizer::options chosen{};
    chosen.root = shared_scratch_directory();

    return chosen;
}

}

#endif
