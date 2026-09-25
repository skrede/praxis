#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_DIFFERENCE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_DIFFERENCE_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/screw_chain.h"

#include "praxis/rigid_motion/types.h"

#include <span>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace praxis::manipulator {

// One joint's entry in a supplied chain: the screw somebody supplied for that joint, or nothing
// where nobody supplied one.
using supplied_screw = std::optional<screw_axis>;

// Which of three answers a joint's line carries. `kinds_differed` is a screw that translates read
// against one that turns, and `not_supplied` is a joint whose entry holds no screw.
enum class chain_joint_reading : std::uint8_t
{
    measured,
    kinds_differed,
    not_supplied
};

// One joint's three terms, each taken after both screws have been divided by their own angular norm
// and none of them summed with another: the angle between the two directions in radians, the greater
// of the two length defects, and the distance between the two moments in metres. Dividing first is
// what keeps the three independent, so an axis of twice the length standing on exactly the right
// line reads its length alone rather than reporting one mistake as three.
//
// A term the comparison does not answer carries no number rather than a zero. A screw whose angular
// part names no axis has no moment to compare, because a translation is the same motion wherever its
// axis is put; a pair whose kinds differ fills no term at all; and a joint whose entry holds nothing
// fills none either. A direction flipped against the one described reads a half turn rather than zero: a
// positive angle about it turns the arm the other way, so it names a different chain rather than the
// same one up to a sign.
struct chain_joint_difference
{
    chain_joint_reading read;
    std::optional<double> direction_radians;
    std::optional<double> length;
    std::optional<double> moment_metres;
};

// How far the supplied home pose stands from the derived one, as three terms carried apart: the
// rotation between them in radians, read after the nearest rotation to the supplied block so that a
// pose written out to a few decimals is answered rather than refused; the distance between their
// origins in metres; and how far either block stands from being a rotation at all, as the greatest
// defect against orthonormality and unit determinant either of them carries.
struct chain_home_difference
{
    double turned_radians;
    double moved_metres;
    double rigidity;
};

// The home line, one line per joint of the derived chain in that chain's own order, and how many
// entries were handed over. A count above the derived chain's length is a surplus the joints do not
// carry; a count below it leaves every joint past the end unsupplied.
struct screw_chain_difference
{
    chain_home_difference home;
    std::vector<chain_joint_difference> joints;
    std::size_t supplied;
};

// A chain somebody supplied read against the one derived from a description, compared directly: no
// pose is taken and no configuration is named, so a joint standing at an angle that hides its own
// screw is read here exactly as every other joint is. An entry holding nothing is a joint nobody
// supplied and carries no term, wherever in the chain it stands. Nothing below judges -- each term is
// reported in its own unit and what it is worth is the caller's to decide. Lynch & Park, Modern
// Robotics, section 3.3.
screw_chain_difference supplied_chain_difference(const screw_chain &derived, const transform &supplied_home, std::span<const supplied_screw> supplied);

}

#endif
