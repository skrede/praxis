#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_AXES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_AXES_H

#include <threepp/core/Object3D.hpp>

#include <memory>
#include <string>
#include <cstdint>

namespace praxis::rigid_motion {

// Lengths are metres, in the z-up frame the object's pose is expressed in. The axis length is the
// whole arrow, tip included.
struct axes_settings
{
    bool shown            = true;
    double axis_length    = 0.5;
    double axis_thickness = 0.05;
};

enum class body_shape : std::uint8_t
{
    none,
    cube,
    mesh
};

struct object_body
{
    body_shape shape = body_shape::cube;
    double cube_edge = 0.25;
    std::shared_ptr<threepp::Object3D> mesh;
};

struct stencil_object
{
    std::string name;
    axes_settings axes;
    object_body body;
};

// The frame every object's chain composes up to, drawn at the scene origin with axes and no body. It
// takes no index among the objects and nothing can move, remove or reassign it. A stencil composed
// with no name for it draws none.
struct fixed_frame
{
    std::string name;
    axes_settings axes;
};

// The origin marker belongs to the axes, so an object with a body carries no marker and its body
// stands in that place. A body of shape none, and a mesh body with nothing to draw, are no node at
// all rather than an empty one.
std::shared_ptr<threepp::Object3D> make_axes(const axes_settings &chosen, bool with_origin_marker);
std::shared_ptr<threepp::Object3D> make_body(const object_body &chosen);

}

#endif
