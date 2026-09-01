#include "evaluation_tables.h"

#include "praxis/rigid_motion/evaluation.h"

#include <array>

namespace praxis::rigid_motion {

std::array<evaluation::evaluation_view, 2> evaluation_views(const capabilities &first, const capabilities &second)
{
    return {evaluation::evaluation_view::of(first.frame, second.frame, frame_evaluations()), evaluation::evaluation_view::of(first.screw, second.screw, screw_evaluations())};
}

}
