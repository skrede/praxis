#ifndef HPP_GUARD_PRAXIS_PRESETS_ROUTES_H
#define HPP_GUARD_PRAXIS_PRESETS_ROUTES_H

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <filesystem>
#include <functional>

namespace praxis::presets {

// Asked where a named document is read from, at the moment a composition wants it. A name can stand
// for more than one candidate place, and which of them answers can change while the application
// runs, so a composition asks again rather than composing through the answer its registration got. A
// route that answers nothing leaves every composition on the location its preset was registered
// with, which is what a caller whose documents have one place each wants.
using document_route = std::function<config::location(const std::filesystem::path &named)>;

// Told what a composition was built from, once that composition has been answered. A caller with
// nowhere to write an edit back to supplies nothing.
using composed_route = std::function<void(const config::binding &, const config::document &)>;

}

#endif
