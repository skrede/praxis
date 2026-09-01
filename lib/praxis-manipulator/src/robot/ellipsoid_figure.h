#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_ELLIPSOID_FIGURE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_ELLIPSOID_FIGURE_H

#include <threepp/core/Object3D.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

// A body whose vertices are rewritten to whatever shape it is next asked for, so it carries a
// geometry of its own rather than one shared with its siblings. Its buffer is wide enough for the
// most vertices any shape can need, and it opens carrying the unit sphere.
std::shared_ptr<threepp::Object3D> ellipsoid_object(std::string name, std::shared_ptr<threepp::Material> tone);

// The surface is the ellipsoid of the three semi-axes, in the object's own frame, intersected with
// the box of half-width cap where a cap is named: the ellipsoid where the cap does not bind and flat
// at |coordinate| == cap on every axis where it does. A semi-axis of zero flattens the body onto the
// plane of the other two. Each flat face lies exactly in its own plane, and the curve where a face
// meets the ellipsoid is an edge of the mesh rather than a chord across a triangle, so the outline
// of a face follows the cut and not the grid. The vertices are rewritten in place and the geometry's
// drawn range names how many of them are in use, so the buffer the renderer holds is never replaced,
// and a call asking for the shape the body already carries does nothing at all.
void shape_ellipsoid(threepp::Object3D &drawn, const Eigen::Vector3d &semi_axes, std::optional<double> cap);

// The columns of axes are the principal directions the semi-axes were given along. The scale is left
// at one, because the lengths live in the vertices: a scale multiplies the flat cut face along with
// everything else and would carry it off the plane it was cut on.
void place_ellipsoid(threepp::Object3D &drawn, const Eigen::Matrix3d &axes, const Eigen::Vector3d &at);

// The line saying which way a truncated principal axis continues past the face it was cut at. Drawn
// from the origin to +Z in its own frame; its placement is what carries it onto the named axis.
std::shared_ptr<threepp::Object3D> continuation_object(std::string name, std::shared_ptr<threepp::Material> tone);

// A length of zero, or a direction naming no axis, is not drawn.
void place_continuation(threepp::Object3D &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length);

// How many steps the condition-number ramp is built at, and which of them a condition number falls
// in: the first step at a condition number of one, the last at the ceiling and above it, and the
// last again where there is no condition number at all.
std::size_t ellipsoid_ramp_steps();
std::size_t ellipsoid_ramp_step(std::optional<double> condition);

// The tone one step of the ramp wears, as a lit body drawn solid or as the same body drawn as a
// wireframe, and as the line continuing a cut axis. Built once per step by the caller and handed to
// every drawing, so a condition number that moves assigns a material rather than building one.
std::shared_ptr<threepp::Material> ellipsoid_material(std::size_t step, bool wireframe);
std::shared_ptr<threepp::Material> continuation_material(std::size_t step);

}

#endif
