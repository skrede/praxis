#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_COLUMN_ARROW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_COLUMN_ARROW_H

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace praxis::manipulator {

// The arrow and the two parts its placement moves independently, so a placement that runs every
// frame never looks a child up by name.
struct drawn_arrow
{
    std::shared_ptr<threepp::Object3D> object;
    std::shared_ptr<threepp::Object3D> shaft;
    std::shared_ptr<threepp::Object3D> tip;
};

// Both parts are built along the renderer's +Y and wear the material given here, so the arrow is
// turned from +Y onto whatever direction it is placed along.
drawn_arrow arrow_object(std::string name, std::shared_ptr<threepp::Material> tone);

// The head keeps its own size at every length, so the shaft carries the whole of the change. A
// length at or below the head's own is drawn as the head alone, shrunk to it. A length of zero, or a
// direction naming no axis, is not drawn.
void place_arrow(const drawn_arrow &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length);

// The tone one part of a Jacobian column wears, plain or highlighted, in the renderer's working color
// space, as every color handed to a material is.
threepp::Color column_tone(jacobian_block part, bool highlighted);

// A material in that tone. Built by the caller once and handed to every arrow of that part, so a
// drawing running every frame assigns a material rather than building one.
std::shared_ptr<threepp::Material> column_material(jacobian_block part, bool highlighted);

// The tone one part of the twist wears, in the renderer's working color space.
threepp::Color twist_tone(jacobian_block part);

// A material in that tone, built by the caller once.
std::shared_ptr<threepp::Material> twist_material(jacobian_block part);

// Chosen by the one frame field that also chooses the ellipsoids' Jacobian. The body Jacobian is
// published at the flange and handed out re-expressed in the tool frame through the tool offset, so
// a body column is a twist of the tool frame.
expected<jacobian, refusal> shown_jacobian(const arm_snapshot &seen, jacobian_frame taken);

}

#endif
