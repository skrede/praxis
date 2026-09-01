#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_JOINT_DECORATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_JOINT_DECORATION_H

#include "praxis/manipulator/types.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>

namespace praxis::manipulator {

// The tone worn by the joint the drawing is about, on its axis line, on its mark and on the segment
// leading to it. One hue serves all three because it is one joint being told apart, so it is stated
// here rather than in either drawing's own file.
inline constexpr threepp::Color::ColorName selected_joint_tone = threepp::Color::magenta;

// Hands a drawn item a material it already has geometry for. An item the renderer draws with no
// material of its own is left alone.
void wear(threepp::Object3D &drawn, const std::shared_ptr<threepp::Material> &tone);

// Proportioned to how far the rendered arm's bounding box is across, so that one opening value
// serves machines of different size. An arm carrying no drawn geometry has no extent to take a
// proportion of, and the stand-in praxis uses there is stated rather than derived.
double opening_axis_reach(threepp::Object3D *arm);

// The tone a drawn axis wears as one of many, and the one it wears while it is the joint the
// drawing is about. Built by the caller once and handed to every line, so a selection that moves
// assigns a material rather than building one.
std::shared_ptr<threepp::Material> axis_material(bool told);

std::shared_ptr<threepp::Object3D> joint_axis_object(std::string name, double reach, std::shared_ptr<threepp::Material> tone);

// The screws are the home axes in the model's root-link frame and the configuration is the one the
// arm is drawn at; joint i's axis is its home screw carried by the product of the exponentials of
// the screws before it. An object whose axis the bound operations decline to carry is not drawn.
void place_joint_axes(std::span<const std::shared_ptr<threepp::Object3D>> drawn, std::span<const screw_axis> space_screws, const joint_vector &theta,
                      const rigid_motion::screw_ops &screw);

// Whether the axes and the chain are withheld rather than placed. Both are placed from a fold
// running through the screw exponential, which carries no refusal channel: a composition that left
// that slot at its default is answered the identity, so the fold reports the home pose whatever the
// arm is doing and nothing placed from it is shown. The slot is named once for as long as it holds
// its default. An arm carrying no chain has nothing folded through it and is left alone.
bool decline_unbound_fold(std::span<const std::shared_ptr<threepp::Object3D>> drawn, threepp::Object3D *chain, const rigid_motion::screw_ops &screw,
                          const rigid_motion::screw_slot_set &inert, bool &reported);

// The pose an object is drawn at, written onto its node. The renderer stores a transform column by
// column and in single precision, which is what the conversion behind this is for.
void write_placement(threepp::Object3D &node, const transform &placed);

}

#endif
