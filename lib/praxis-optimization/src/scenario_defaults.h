#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H

#include <cstddef>
#include <cstdint>

namespace praxis::optimization {

// The |g(x)| a curve's solve stops below, in the units of g.
inline constexpr double curve_tolerance = 1.0e-12;

// The |f(x, y)| a surface's solve stops below, in the units of f.
inline constexpr double surface_tolerance = 1.0e-12;

// The |f'(x)| a curve's minimization stops below, in the units of f per unit of x.
inline constexpr double curve_objective_tolerance = 1.0e-12;

// The largest |df/dx|, |df/dy| a surface's minimization stops below, in the units of f per unit of x.
inline constexpr double surface_objective_tolerance = 1.0e-12;

// The value a letter new to a typed function takes, and the minimum and the maximum of the range it
// is chosen from, in whatever units the function gives the letter.
inline constexpr double parameter_value = 1.0;
inline constexpr double parameter_lower = -5.0;
inline constexpr double parameter_upper = 5.0;

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

// An iterate whose center stands within this distance of one drawn before it is not drawn, unless it is
// the run's last or the selected step's, in meters.
inline constexpr double iterate_spacing = 0.75 * iterate_radius;

// The half-side of the tangent plane drawn at a step from x, as a multiple of the distance from x to
// the plane's zero line, dimensionless.
inline constexpr double tangent_patch = 1.25;

// The level curves a minimized surface's map draws, a count, at the k/(count + 1) quantiles of the
// grid's finite samples.
inline constexpr std::size_t level_count = 16;

// The distance the map's floor stands below the field's lowest drawn point, a fraction of the extent.
inline constexpr double floor_gap = 0.1;

// The length the gradient arrow is drawn at whatever |grad f|, a fraction of the extent.
inline constexpr double gradient_arrow_fraction = 0.2;

// The radius of the gradient arrow's shaft, and the radius and the length of its head, in meters.
inline constexpr double gradient_shaft_radius = iterate_radius / 4.0;
inline constexpr double gradient_head_radius  = iterate_radius;
inline constexpr double gradient_head_length  = 2.0 * iterate_radius;

// The radius of the rod the step taken on a minimized surface is drawn as, and the height its axis
// stands above the floor with the floor projection on, in meters.
inline constexpr double floor_step_radius = 2.0 * iterate_radius / 3.0;
inline constexpr double floor_step_lift   = iterate_radius + floor_step_radius;

// The opacity the field and the linearization planes are drawn at over a minimized surface's map, from
// 0 (unseen) to 1 (opaque).
inline constexpr double minimum_field_opacity = 0.8;
inline constexpr double minimum_plane_opacity = 0.6;

// Where the camera stands when a surface opens, turning about the origin of the scene's y-up frame:
// its elevation above the floor's plane and its azimuth about the vertical from +z toward +x, in
// radians, and its distance from the origin, in meters.
inline constexpr double opening_elevation_radians = 0.9599310885968813;
inline constexpr double opening_azimuth_radians   = 0.0;
inline constexpr double opening_distance          = 1.6;

// The tones a surface, a curve and what is drawn on them wear, as 0xRRGGBB. The tangent at x_k wears
// tangent_tone, and its zero, from which x_(k+1) is drawn, tangent_zero_tone.
inline constexpr std::uint32_t surface_tone          = 0x4682b4;
inline constexpr std::uint32_t curve_tone            = 0x6495ed;
inline constexpr std::uint32_t iterate_tone          = 0xff4500;
inline constexpr std::uint32_t selected_iterate_tone = 0xff00ff;
inline constexpr std::uint32_t answer_tone           = 0x00ffff;
inline constexpr std::uint32_t zero_curve_tone       = 0x0000cd;
inline constexpr std::uint32_t tangent_tone          = 0xf0e68c;
inline constexpr std::uint32_t tangent_zero_tone     = 0x008080;
inline constexpr std::uint32_t box_tone              = 0x4b0082;
inline constexpr std::uint32_t level_curve_tone      = 0x0000cd;
inline constexpr std::uint32_t floor_iterate_tone    = 0x000000;
inline constexpr std::uint32_t gradient_tone         = 0xffd700;
inline constexpr std::uint32_t floor_step_tone       = 0x8b0000;

// The tone of the line drawn on a surface from the zero plane up to x_(k+1), as 0xRRGGBB.
inline constexpr std::uint32_t next_iterate_line_tone = tangent_zero_tone;

// The tone the floor marker of the selected step's iterate wears, as 0xRRGGBB.
inline constexpr std::uint32_t floor_selected_iterate_tone = 0x00ffff;

// The tone of the linearization at p, as 0xRRGGBB: a curve's point and its tangent, and a surface's
// plane at p.
inline constexpr std::uint32_t linearization_point_tone = 0xffffff;

// The tone the central difference at p wears, as 0xRRGGBB: a line on a curve, a wire plane on a surface.
inline constexpr std::uint32_t central_difference_tone = 0x00ffff;

// |d - d_cd| / max(1, |d|, |d_cd|) above which a registered derivative d and the central difference
// d_cd of the value disagree, a dimensionless relative difference.
inline constexpr double derivative_check_tolerance = 1.0e-7;

}

#endif
