#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_VIEW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_VIEW_H

namespace praxis::optimization {

// What a scenario's drawing shows, written by the windows that switch it and read by the curve's plot
// or the surface's stencil, all on the strand the frames are drawn on. A curve has no field, no zero
// curve, no level curves and no gradient arrow, so it reads none of them; a surface draws its zero
// curve in the root form, and in the minimum form its level curves, gradient and step on the field,
// or with the floor projection on a floor beneath the field with its iterates projected there.
struct scenario_view
{
    scenario_view()
            : field(true)
            , iterates(true)
            , zero_curve(true)
            , level_curves(true)
            , gradient(true)
            , step(true)
            , bounds(true)
            , linearization(true)
            , floor_projection(false)
    {
    }

    bool field;
    bool iterates;
    bool zero_curve;
    bool level_curves;
    bool gradient;
    bool step;
    bool bounds;
    bool linearization;
    bool floor_projection;
};

}

#endif
