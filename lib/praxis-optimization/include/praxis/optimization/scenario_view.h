#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_VIEW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_VIEW_H

namespace praxis::optimization {

// What a scenario's drawing shows, written by the windows that switch it and read by the curve's plot
// or the surface's stencil, all on the strand the frames are drawn on. A curve has no field and no
// zero curve, so it reads neither.
struct scenario_view
{
    scenario_view()
            : field(true)
            , iterates(true)
            , zero_curve(true)
            , step(true)
            , bounds(true)
            , linearization(true)
    {
    }

    bool field;
    bool iterates;
    bool zero_curve;
    bool step;
    bool bounds;
    bool linearization;
};

}

#endif
