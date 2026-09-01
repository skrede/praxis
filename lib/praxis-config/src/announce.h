#ifndef HPP_GUARD_PRAXIS_CONFIG_ANNOUNCE_H
#define HPP_GUARD_PRAXIS_CONFIG_ANNOUNCE_H

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

namespace praxis::config {

void report(const location &at);

void announce_refusal(const location &at, const error &refused, expectation carries);

void announce_substitutions(const declaration &shape, const document &values, expectation carries);

}

#endif
