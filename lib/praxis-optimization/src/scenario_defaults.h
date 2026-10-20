#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H

#include <cstddef>
#include <cstdint>

namespace praxis::optimization {

// The |g(x)| a curve's solve stops below, in the units of g.
inline constexpr double curve_tolerance = 1.0e-12;

// The |f(x, y)| a surface's solve stops below, in the units of f.
inline constexpr double surface_tolerance = 1.0e-12;

// The points a curve is sampled at over its range, ends included, a count.
inline constexpr std::size_t curve_samples = 1601;

// The points a surface is sampled at along each of its two ranges, ends included, a count.
inline constexpr std::size_t surface_samples = 128;

// The side of the square in the scene the wider of a surface's two ranges is drawn across, in meters.
inline constexpr double surface_extent = 1.25;

// The height the largest |f| over a surface's grid is drawn at, a fraction of the extent.
inline constexpr double surface_height = 0.5;

// The radius of the sphere an iterate is drawn as on a surface, in meters.
inline constexpr double iterate_radius = 0.015;

// The tones a surface and what is drawn on it wear, as 0xRRGGBB.
inline constexpr std::uint32_t surface_tone          = 0x4682b4;
inline constexpr std::uint32_t iterate_tone          = 0xff4500;
inline constexpr std::uint32_t selected_iterate_tone = 0xff00ff;
inline constexpr std::uint32_t zero_curve_tone       = 0x0000cd;
inline constexpr std::uint32_t tangent_plane_tone    = 0xf0e68c;
inline constexpr std::uint32_t zero_line_tone        = 0x000080;
inline constexpr std::uint32_t box_tone              = 0xa0522d;

// |d - d_cd| / max(1, |d|, |d_cd|) above which a registered derivative d and the central difference
// d_cd of the value disagree, a dimensionless relative difference.
inline constexpr double derivative_check_tolerance = 1.0e-7;

}

#endif
