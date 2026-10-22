#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_ARROW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_ARROW_H

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <cstdint>

namespace praxis::optimization {

// An arrow of a shaft `gradient_shaft_radius` across and a head `gradient_head_radius` across and
// `gradient_head_length` long, both built along +y and lit in one tone, so the arrow is turned from +y
// onto whatever direction it is placed along.
struct surface_arrow
{
    std::shared_ptr<threepp::Group> object;
    std::shared_ptr<threepp::Mesh> shaft;
    std::shared_ptr<threepp::Mesh> head;
};

surface_arrow arrow_object(std::string name, std::uint32_t tone);

// The head keeps its own size at every length, so the shaft carries the whole of the change; a length
// at or below the head's own is drawn as the head alone, shrunk to it.
void place_arrow(const surface_arrow &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length);

}

#endif
