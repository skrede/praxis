#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_CONFIGURATION_H

#include "praxis/rigid_motion/frame_window.h"

#include "praxis/config/error.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::rigid_motion {

// `at` is a key path the caller owns. This declares one instance per object beneath it, keyed by the
// object's own name, and declares nothing above it. `objects` is those names in the arrangement's
// own order, and is what a name in the document is turned back into an index against.
//
// An absent parent key means the object is expressed in the world frame.

void declare_arrangement(config::declaration &shape, std::string_view at);

// A parent naming no object, and a chain of parents that would close a cycle, are each refused with
// the offending name carried in the message. `opening` is the arrangement each object's placement is
// read against, so an object the document names no instance of keeps the placement the caller
// supplied for it.
expected<frame_window::settings, config::error> read_arrangement(const config::document &values, std::string_view at, std::span<const std::string> objects,
                                                                 const frame_window::settings &opening = frame_window::settings());

// `opened` is the arrangement the composition was built with. Only the leaves of a placement that
// stand differently from where `values` and `opened` together put them are written, and a leaf left
// out is one the document is left to not carry. An object the document carries no instance for is
// written as a new instance at the end of the collection, named by the identity edit that precedes
// the leaves that moved, so no ordinal an existing instance is addressed at moves.
std::vector<config::edit> write_arrangement(const config::document &values, const frame_window::settings &opened, const frame_window::settings &state, std::string_view at,
                                            std::span<const std::string> objects);

}

#endif
