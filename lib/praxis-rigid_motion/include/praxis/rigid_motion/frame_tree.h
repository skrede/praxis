#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_TREE_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_FRAME_TREE_H

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::rigid_motion {

// Which frame each frame's placement is expressed in. An absent parent means the tree's root frame.
// An index past the end reads as the identity and is a no-op on the setters. A removal compacts
// rather than leaving a hole, so an index naming a frame after the removed one names that same frame
// one lower afterwards; the tree carries the indices it holds itself across that shift, and an index
// held elsewhere is the holder's to carry.
class frame_tree
{
public:
    frame_tree(std::size_t frames, frame_ops motions);

    std::size_t count() const;

    // pose is the placement in the parent; world_pose is that chain composed up to the root frame.
    const transform &pose(std::size_t index) const;
    transform world_pose(std::size_t index) const;

    std::optional<std::size_t> parent_of(std::size_t index) const;

    void set_pose(std::size_t index, const transform &tf);

    // A self-parent, a parent already descending from the child, and an index past the end are each
    // refused and leave the relation as it was, so a chain over N frames has at most N links.
    expected<void, refusal> set_parent(std::size_t child, std::optional<std::size_t> parent);

    // A frame joins with the identity placement and no parent, at the index the call answers. An
    // index past the end is refused as unsupported input; an index another frame's placement is
    // expressed in is refused as having no solution, since compaction cannot carry a parent whose
    // frame is gone. Either refusal leaves the frame set as it was.
    std::size_t add();
    expected<void, refusal> remove(std::size_t index);

private:
    struct link
    {
        transform placement;
        std::optional<std::size_t> parent;
    };

    std::vector<link> m_links;

    transform root_pose(std::size_t index) const;
    bool descends_from(std::size_t node, std::size_t ancestor) const;
    bool expressed_in(std::size_t index) const;
    void carry_indices_across(std::size_t removed);
};

}

#endif
