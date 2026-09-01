#ifndef HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_ROTATION_ARROW_H
#define HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_ROTATION_ARROW_H

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace praxis::presets {

struct drawn_arrow
{
    std::shared_ptr<threepp::Object3D> object;
    std::shared_ptr<threepp::Object3D> stem;
    std::shared_ptr<threepp::Object3D> head;
};

// Both parts are built along the renderer's +Y, and the group carrying them is turned from +Y onto
// the direction it is placed along. The girth is the stem's diameter in metres.
drawn_arrow arrow_object(std::string name, const threepp::Color &tone, double girth);

// The vector the arrow stands for, in metres in the z-up frame the stencil's poses are written in:
// its length is the length drawn and its direction the direction turned onto. A vector of zero
// length is not drawn.
void place_arrow(const drawn_arrow &drawn, const Eigen::Vector3d &along);

}

#endif
