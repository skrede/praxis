#include "praxis/scene/preset_registry.h"

namespace praxis::probe {

namespace {

const bool registered = (scene::preset_registry::register_preset("registered before main", nullptr), true);

}

}
