#include "surface_step.h"
#include "drawn_iterates.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <functional>
#include <unordered_map>

namespace praxis::optimization {

namespace {

// A cell's coordinates are floored multiples of its side kept as doubles, since a placed height is
// not held to the grid's and may lie past any integer's range.
struct cell
{
    double x;
    double y;
    double z;

    bool operator==(const cell &other) const = default;
};

// boost::hash_combine's mix over the three coordinates.
struct cell_hash
{
    std::size_t operator()(const cell &at) const
    {
        std::size_t mixed = 0u;
        for(const double coordinate : {at.x, at.y, at.z})
            mixed ^= std::hash<double>{}(coordinate) + 0x9e3779b97f4a7c15ull + (mixed << 6) + (mixed >> 2);

        return mixed;
    }
};

using held_centers = std::unordered_map<cell, std::vector<Eigen::Vector3d>, cell_hash>;

// Adding zero turns a floored -0 into +0, which hashes as the +0 it compares equal to.
cell cell_of(const Eigen::Vector3d &at, double side)
{
    return cell{std::floor(at.x() / side) + 0.0, std::floor(at.y() / side) + 0.0, std::floor(at.z() / side) + 0.0};
}

bool held_within(const held_centers &held, const cell &looked, const Eigen::Vector3d &at, double apart)
{
    const auto found = held.find(looked);

    return found != held.end() && std::ranges::any_of(found->second, [&](const Eigen::Vector3d &center) { return (center - at).squaredNorm() < apart * apart; });
}

// A center within `apart` lies in the cell of `at` or in one of the 26 around it.
bool near_held(const held_centers &held, const Eigen::Vector3d &at, double apart)
{
    const cell own = cell_of(at, apart);
    if(held_within(held, own, at, apart))
        return true;

    constexpr std::array<double, 3> offsets{-1.0, 0.0, 1.0};
    for(const double dx : offsets)
        for(const double dy : offsets)
            for(const double dz : offsets)
                if((dx != 0.0 || dy != 0.0 || dz != 0.0) && held_within(held, cell{own.x + dx, own.y + dy, own.z + dz}, at, apart))
                    return true;

    return false;
}

}

std::vector<drawn_iterate> drawn_iterates(const scenario_snapshot &shown, const field_placement &placed, double apart)
{
    const scenario_run &run = shown.run;
    std::vector<drawn_iterate> drawn;
    held_centers held;
    for(std::size_t k = 0; k < run.iterates.size(); ++k)
    {
        if(!within_ranges(shown.ranges, run.iterates[k]) || k >= run.errors.size() || !run.errors[k].has_value())
            continue;

        const Eigen::Vector3d at = placed.at(run.iterates[k][0], run.iterates[k][1], -(*run.errors[k])[0]);
        const bool kept          = k + 1u == run.iterates.size() || k == shown.step;
        if(!kept && near_held(held, at, apart))
            continue;

        held[cell_of(at, apart)].push_back(at);
        drawn.push_back(drawn_iterate{k, at});
    }
    const auto selected = std::ranges::find_if(drawn, [&shown](const drawn_iterate &mark) { return mark.iterate == shown.step; });
    if(selected != drawn.end())
        std::rotate(selected, selected + 1, drawn.end());

    return drawn;
}

}
