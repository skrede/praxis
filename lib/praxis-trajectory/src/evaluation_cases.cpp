#include "evaluation_cases.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <span>
#include <array>
#include <cstddef>
#include <numbers>
#include <utility>
#include <algorithm>

namespace praxis::trajectory {

namespace {

// Seconds.
constexpr double shortest_duration = 0.25;

constexpr std::array polynomial_kinks{snapped_instant::start, snapped_instant::finish};

}

// Under `near_singular` the draw lands a little below zero.
snapped_instant snapped_among(evaluation::case_source &drawn, std::span<const snapped_instant> instants, double fraction)
{
    const double unit = over(drawn, 0.0, 1.0);
    if(!(unit < fraction))
        return snapped_instant::drawn;

    const double last  = static_cast<double>(instants.size() - 1u);
    const double which = std::clamp(unit / fraction * static_cast<double>(instants.size()), 0.0, last);

    return instants[static_cast<std::size_t>(which)];
}

double snapped_time(snapped_instant instant, double drawn_at, double duration, double ramp)
{
    switch(instant)
    {
        case snapped_instant::start:
            return 0.0;
        case snapped_instant::ramp_end:
            return ramp;
        case snapped_instant::coast_end:
            return duration - ramp;
        case snapped_instant::finish:
            return duration;
        case snapped_instant::drawn:
            break;
    }

    return drawn_at;
}

double over(evaluation::case_source &drawn, double from, double to)
{
    const double unit = 0.5 * (1.0 + drawn.angle_radians() / std::numbers::pi_v<double>);

    return from + (to - from) * unit;
}

configuration drawn_configuration(evaluation::case_source &drawn, std::size_t coordinates)
{
    configuration values(static_cast<Eigen::Index>(coordinates));

    for(Eigen::Index axis = 0; axis < values.size(); ++axis)
        values[axis] = drawn.angle_radians();

    return values;
}

scaling_case drawn_scaling_case(evaluation::case_source &drawn)
{
    const double duration         = over(drawn, shortest_duration, longest_duration);
    const double at               = over(drawn, 0.0, duration);
    const double speed            = over(drawn, slowest_speed_bound, fastest_speed_bound);
    const double acceleration     = over(drawn, gentlest_acceleration_bound, harshest_acceleration_bound);
    const snapped_instant snapped = snapped_among(drawn, polynomial_kinks, snap_fraction);

    return scaling_case{duration, snapped_time(snapped, at, duration, 0.0), speed, acceleration, snapped};
}

joint_path_case drawn_joint_path_case(evaluation::case_source &drawn)
{
    const std::size_t coordinates = drawn.axis_count();
    configuration start           = drawn_configuration(drawn, coordinates);
    configuration end             = drawn_configuration(drawn, coordinates);

    return joint_path_case{std::move(start), std::move(end), over(drawn, 0.0, 1.0)};
}

pose_path_case drawn_pose_path_case(evaluation::case_source &drawn)
{
    const transform start = drawn.transform_member();
    const transform end   = drawn.transform_member();

    return pose_path_case{start, end, over(drawn, 0.0, 1.0)};
}

const rigid_motion::screw_ops &shared_screw()
{
    static const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    return screw;
}

const rigid_motion::frame_ops &shared_frames()
{
    static const rigid_motion::frame_ops frames = rigid_motion::baseline().frame;

    return frames;
}

const path_ops &shared_paths()
{
    static const path_ops paths = baseline().path;

    return paths;
}

const time_scaling_ops &shared_time_scalings()
{
    static const time_scaling_ops scalings = baseline().time_scaling;

    return scalings;
}

}
