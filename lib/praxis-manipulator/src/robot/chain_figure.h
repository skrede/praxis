#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CHAIN_FIGURE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CHAIN_FIGURE_H

#include <threepp/core/Object3D.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>

namespace praxis::manipulator {

// The tone a chain item wears as one of many, and the one it wears while it belongs to the joint the
// drawing is about. Built by the caller once and handed to every item, so a selection that moves
// assigns a material rather than building one.
std::shared_ptr<threepp::Material> chain_material(bool told);

// The tone the same items wear while they stand at a configuration the arm is not at.
std::shared_ptr<threepp::Material> solution_material();

// One segment of the chain, built at unit height along the renderer's +Y and centered on its own
// origin, so that scaling it along its own y is what gives it the length it has to span.
std::shared_ptr<threepp::Object3D> chain_segment_object(std::string name, std::shared_ptr<threepp::Material> tone);

// The mark standing at one joint origin.
std::shared_ptr<threepp::Object3D> joint_mark_object(std::string name, std::shared_ptr<threepp::Material> tone);

// Segment i spans points i and i+1: it sits at their midpoint, is turned from the renderer's +Y onto
// the direction from the first to the second, and is scaled along its own y by the distance between
// them. Two points that coincide give a segment of no length, placed at that point and not drawn.
// Mark j stands at the origin of joint j, which is the point after the frame's own.
void place_chain_figure(std::span<const std::shared_ptr<threepp::Object3D>> segments, std::span<const std::shared_ptr<threepp::Object3D>> marks,
                        std::span<const Eigen::Vector3d> points);

}

#endif
