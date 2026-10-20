#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_PROJECTED_PERIODIC_BOUNDS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_PROJECTED_PERIODIC_BOUNDS_H

#include <optional>

namespace praxis::optimization::projected {

// value + k period inside [lower, upper], k nearest zero; a period of zero admits the value alone.
std::optional<double> named_between(double value, double lower, double upper, double period);

// The projection of Kanzow, Yamashita & Fukushima, J. Comput. Appl. Math. 172, 2004, onto one
// coordinate: its naming inside the bounds when one exists, else, wrapping, the bound nearer on the
// circle (a tie to the bound crossed), else the bound crossed.
double projected_between(double value, double lower, double upper, double period);

}

#endif
