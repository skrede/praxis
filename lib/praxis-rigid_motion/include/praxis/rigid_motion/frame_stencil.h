#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_STENCIL_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_STENCIL_H

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_tree.h"

#include "praxis/scene/stencil.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <threepp/math/Box3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <vector>
#include <cstddef>
#include <optional>
#include <functional>
#include <string_view>

namespace praxis::rigid_motion {

// A pose here is expressed in a z-up frame and the renderer's world is y-up, so every object hangs
// under a root node carrying that quarter turn rather than every pose being converted where it is
// set. Objects sit flat under that one root: the parent relation is composed into the poses written
// here, never into the renderer's node hierarchy.
//
// The fixed frame is the frame those chains compose up to. It hangs under the same root, so it stands
// at the scene origin, and it is not one of the objects the accessors below index: it takes no index,
// it is not counted, and nothing can move or remove it.
//
// Where two or more objects stand here one of them is marked, the one a selection names, and where a
// lone object stands none is: a mark tells one object from the others and there are none. The mark is
// a box taken from the whole extent of that object's node, so an object carrying no body is marked
// like any other, and no material anywhere is written to draw it.
class frame_stencil : public scene::stencil
{
public:
    frame_stencil(threepp::Scene &parent, std::vector<stencil_object> objects, frame_ops motions, fixed_frame anchored = fixed_frame());
    ~frame_stencil() override = default;

    expected<void, refusal> initialize() override;
    void tear_down() override;

    void render() const override;

    std::size_t count() const;

    // The name the composition gave the fixed frame, empty where it named none.
    std::string_view fixed_frame_name() const
    {
        return m_fixed.name;
    }

    // An index past the end reads as the identity pose, an empty name and hidden axes, marks
    // nothing, and is a no-op on the setters, so a control cannot index outside the object set.
    bool axes_shown(std::size_t index) const;
    const transform &pose(std::size_t index) const;
    std::string_view name_of(std::size_t index) const;

    transform world_pose(std::size_t index) const;
    std::optional<std::size_t> parent_of(std::size_t index) const;

    void set_pose(std::size_t index, const transform &tf);
    void set_body(std::size_t index, object_body chosen);

    // Visibility is a view of an object rather than part of its placement, so hiding an object's
    // axes moves nothing an arrangement carries and leaves the nodes the stencil hung in place.
    void set_axes_shown(std::size_t index, bool shown);

    expected<void, refusal> set_parent(std::size_t child, std::optional<std::size_t> parent);

    // The object set and the frame set grow and shrink together, so a removal refuses on exactly the
    // conditions the frame tree states and leaves the scene untouched where it does.
    std::size_t add(stencil_object described);
    expected<void, refusal> remove(std::size_t index);

    // Which object the mark is drawn on, asked once per drawn frame. A stencil handed no source
    // marks nothing.
    void follow_selection(std::function<std::size_t()> selecting);

private:
    struct placed
    {
        stencil_object described;
        std::shared_ptr<threepp::Object3D> node;
        std::shared_ptr<threepp::Object3D> axes;
        std::shared_ptr<threepp::Object3D> body;
    };

    frame_tree m_tree;
    fixed_frame m_fixed;
    threepp::Scene &m_scene;
    std::vector<placed> m_objects;
    std::shared_ptr<threepp::Object3D> m_root;
    std::shared_ptr<threepp::Object3D> m_anchor;

    // Declared ahead of the marker, which holds this box by reference for as long as it lives, and
    // written while drawing, which the base class asks for on a const stencil.
    mutable threepp::Box3 m_marked;
    std::shared_ptr<threepp::Object3D> m_marker;
    std::function<std::size_t()> m_selecting;

    void attach_axes(placed &object);
    void attach_body(placed &object);
    void withdraw(std::size_t index);

    void raise_marker();
    void draw_mark() const;
};

}

#endif
