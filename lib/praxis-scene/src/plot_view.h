#ifndef HPP_GUARD_PRAXIS_SCENE_PLOT_VIEW_H
#define HPP_GUARD_PRAXIS_SCENE_PLOT_VIEW_H

#include "praxis/scene/plot_window.h"

#include <optional>

namespace praxis::scene {

// The view of a frame carrying held limits: absent while it follows the fitted view those limits
// give, and the limits it was fitted to once the pointer has moved it away.
using plot_view = std::optional<plot_limits>;

bool returns_to_fitted(const plot_limits &moved_from, const plot_limits &held, bool double_clicked);

// Whether the frame is set up at its fitted view this frame. The pointer is read before the plot's
// axes are set up and against the plot as the previous frame left it, because an axis set up at the
// fitted view takes no input in the frame it is set up in.
bool shows_fitted(plot_view &view, const plot_frame &shown, const char *plot_id);

}

#endif
