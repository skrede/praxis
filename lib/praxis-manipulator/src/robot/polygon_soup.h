#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_POLYGON_SOUP_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_POLYGON_SOUP_H

#include <Eigen/Core>

#include <span>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// A triangle soup being written into a caller's span, three floats a corner and three corners a
// triangle, and how many floats of it are in use. A corner past the end of the span is dropped
// rather than written.
struct polygon_soup
{
    std::span<float> into;
    std::size_t written;
};

// The polygon a clip is working on and the one it is building, held together so that a run of
// clips allocates only while the first few of them are widening the two.
struct polygon_work
{
    std::vector<Eigen::Vector3d> shape;
    std::vector<Eigen::Vector3d> kept;
};

void put_soup_corner(polygon_soup &out, const Eigen::Vector3d &corner);

// Sutherland and Hodgman's clip of the polygon in hand against out.dot(corner) <= reach, keeping
// the part inside. A convex polygon stays convex under it and keeps its winding.
void clip_polygon(polygon_work &work, const Eigen::Vector3d &out, double reach);

// The polygon in hand fanned from its own first corner, which a convex polygon always admits.
void put_polygon(polygon_soup &out, const polygon_work &work);

}

#endif
