#ifndef HPP_GUARD_PRAXIS_SCENE_STENCIL_H
#define HPP_GUARD_PRAXIS_SCENE_STENCIL_H

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

namespace praxis::scene {

class stencil
{
public:
    stencil()                           = default;
    stencil(const stencil &)            = delete;
    stencil(stencil &&)                 = delete;
    stencil &operator=(const stencil &) = delete;
    stencil &operator=(stencil &&)      = delete;
    virtual ~stencil()                  = default;

    virtual expected<void, refusal> initialize() = 0;
    virtual void tear_down()                     = 0;

    virtual void render() const = 0;
};

}

#endif
