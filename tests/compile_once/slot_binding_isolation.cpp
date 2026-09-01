#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"

#include "praxis/trajectory/slots.h"
#include "praxis/trajectory/types.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/types.h"

#if defined(HPP_GUARD_PRAXIS_SCHEDULER_STRAND_H) || defined(HPP_GUARD_PRAXIS_SCHEDULER_SCHEDULER_H) || defined(HPP_GUARD_PRAXIS_SCENE_VISUALIZER_H) ||                                  \
        defined(HPP_GUARD_PRAXIS_SCENE_IMGUI_WINDOW_H)
    #error "a slot table pulled in a scheduler or a scene header"
#endif
#if __has_include(<praxis/scheduler/strand.h>) || __has_include(<praxis/scene/imgui_window.h>)
    #error "the scheduler's or the scene's include directory reached a translation unit that only binds slots"
#endif

#include <span>

namespace robotics_course {

// One slot of each extension, in the shape a consumer writes: a free function whose signature
// carries robotics and nothing else, composed into the aggregate that decides the arity and the
// order. A signature that named a scheduling type would still compile here; what this translation
// unit answers is whether the headers reached to write one can see the scheduler at all.
praxis::rotation rotate_z(double radians)
{
    static_cast<void>(radians);

    return praxis::rotation::Identity();
}

praxis::expected<praxis::trajectory::scaling_sample, praxis::refusal> cubic(double t, double duration)
{
    if(duration <= 0.0)
        return praxis::unexpected(praxis::refusal::degenerate);

    return praxis::trajectory::scaling_sample{t / duration, 1.0 / duration, 0.0};
}

praxis::expected<praxis::transform, praxis::refusal> forward_kinematics(const praxis::transform &m, std::span<const praxis::screw_axis>, const praxis::manipulator::joint_vector &)
{
    return m;
}

praxis::rigid_motion::frame_ops frames()
{
    return praxis::rigid_motion::frame_ops{.rotate_z = &rotate_z};
}

praxis::trajectory::time_scaling_ops scalings()
{
    return praxis::trajectory::time_scaling_ops{.cubic = &cubic};
}

praxis::manipulator::forward_kinematics_ops forward_maps()
{
    return praxis::manipulator::forward_kinematics_ops{.forward_kinematics = &forward_kinematics};
}

}
