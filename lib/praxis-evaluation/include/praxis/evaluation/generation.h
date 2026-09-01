#ifndef HPP_GUARD_PRAXIS_EVALUATION_GENERATION_H
#define HPP_GUARD_PRAXIS_EVALUATION_GENERATION_H

#include <Eigen/Core>

#include <random>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::evaluation {

// Which part of a role's range a draw comes from. Under `near_singular` a routine whose role has a
// neighbourhood where the mathematics degenerates draws from that neighbourhood instead of from the
// bulk of the range. `unit_direction`, `position_metres`, `angular_part`, `linear_part`,
// `normal_triple`, `skew_symmetric_member`, `twist_member` and `axis_order_index` have no such
// neighbourhood and draw from the same distribution under both.
enum class spread : std::uint8_t
{
    bulk,
    near_singular
};

// A source draws a sequence fixed entirely by the seed, the slot name, the spread and the case index
// it was built with, so a slot's inputs do not move when another slot is added to, removed from or
// reordered within a table. Nothing here reads a clock or any other entropy source: a report's
// `seed` together with a slot report's `worst_case_index` is enough to redraw the exact input a
// failure was found at.
class case_source
{
public:
    explicit case_source(std::uint64_t seed, spread drawn_from = spread::bulk);

    static case_source for_slot(std::uint64_t seed, std::string_view slot, spread drawn_from = spread::bulk);

    // The source the case at `index` drew from, reached without replaying the cases before it.
    // `for_slot` is the case at index zero.
    static case_source at_case(std::uint64_t seed, std::string_view slot, spread drawn_from, std::size_t index);

    std::uint64_t seed() const;
    spread drawn_from() const;

    // Uniform over a full turn, in radians. Under `near_singular`, within one radian of zero or of a
    // half turn and no nearer either than the comparison tolerance.
    double angle_radians();

    // Axial travel per radian, in metres, uniform over a range straddling zero, so a motion the
    // rotation dominates and one the travel dominates are both drawn. Under `near_singular`, within
    // one metre per radian of zero and no nearer it than the comparison tolerance.
    double pitch();

    // Uniform over the sphere: three standard normals normalized, redrawn while the norm stands
    // under the comparison tolerance. Normalizing a draw from a box would crowd the cube's corners.
    Eigen::Vector3d unit_direction();

    // Every component uniform over the same range, in metres.
    Eigen::Vector3d position_metres();

    // Three angles in radians, the outer two uniform over a full turn. Under `near_singular` the
    // middle one lies within a radian of zero, of a quarter turn either way, or of a half turn --
    // the values at which a rotation order loses a degree of freedom.
    Eigen::Vector3d euler_triple_radians();

    // Exactly zero half the time and three standard normals otherwise, so the case of a motion with
    // no rotation at all is reached.
    Eigen::Vector3d angular_part();

    // Three standard normals.
    Eigen::Vector3d linear_part();

    // Three standard normals, the unconstrained triple every other three-vector role is built from.
    Eigen::Vector3d normal_triple();

    // The three columns of a `rotation_member`, each of unit norm, mutually orthogonal and
    // right-handed: the third column is the cross product of the first two.
    Eigen::Matrix3d orthonormal_triple();

    // An `angle_radians` turn about a `unit_direction`, so the transpose times itself is the
    // identity and the determinant is one.
    Eigen::Matrix3d rotation_member();

    // A `rotation_member` over a `position_metres`, under a bottom row of zero, zero, zero, one.
    Eigen::Matrix4d transform_member();

    // The matrix of the cross product with a `normal_triple`, so the matrix added to its own
    // transpose is exactly the zero matrix.
    Eigen::Matrix3d skew_symmetric_member();

    // An angular part of unit norm over an unconstrained linear part, or -- half the time, and
    // always under `near_singular` -- an angular part of exactly zero over a linear part of unit
    // norm. Never entirely zero.
    Eigen::Vector<double, 6> unit_twist();

    // Six standard normals.
    Eigen::Vector<double, 6> twist_member();

    // Uniform over the closed range zero to eleven, the orderings three rotation axes admit.
    std::uint8_t axis_order_index();

    // Uniform over the closed range one to eight, the lengths a drawn sequence of unit twists takes.
    std::size_t axis_count();

private:
    spread m_spread;
    std::uint64_t m_seed;
    std::mt19937_64 m_engine;

    bool half_the_time();
    double standard_normal();
    double offset_from_singular();
};

}

#endif
