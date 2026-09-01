#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_DRAWN_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_DRAWN_CHAIN_H

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <functional>
#include <string_view>

namespace praxis::fixture {

using named_by_index = std::function<std::string(std::size_t)>;

// Found by the name the chain hangs under rather than by taking the first group a traverse reaches:
// a scene carries more than one group and traverse order does not say which is which.
inline threepp::Object3D *chain_node(threepp::Scene &target, std::string_view named)
{
    return target.getObjectByName<threepp::Object3D>(std::string(named));
}

inline threepp::Object3D *chain_part(threepp::Scene &target, std::string_view chain, std::string_view part)
{
    threepp::Object3D *under = chain_node(target, chain);

    return under == nullptr ? nullptr : under->getObjectByName<threepp::Object3D>(std::string(part));
}

// A point of the object's own +Y axis, in the frame the poses are written in. The renderer's world
// is y-up and every object hangs under a root carrying that quarter turn, so a world reading is
// turned back through it.
inline Eigen::Vector3d carried_out(threepp::Object3D &drawn, float along)
{
    threepp::Vector3 put{0.f, along, 0.f};
    drawn.localToWorld(put);

    return {put.x, -put.z, put.y};
}

// A segment is a unit-height primitive along its own +Y scaled by the length it spans, so its two
// ends are its own (0, -1/2, 0) and (0, 1/2, 0) carried out.
inline std::vector<Eigen::Vector3d> segment_ends(threepp::Object3D &drawn)
{
    return {carried_out(drawn, -0.5f), carried_out(drawn, 0.5f)};
}

// A joint mark takes no turn and no length, so it stands where its own origin is carried to.
inline Eigen::Vector3d mark_in_world(threepp::Object3D &drawn)
{
    return carried_out(drawn, 0.f);
}

// The points the chain is drawn through, recovered from where its segments were placed: the first
// end of the first segment, and then the second end of each. A segment of no length answers its own
// point twice, so a pair of axes that meet reads back as two coincident points rather than as one.
inline std::vector<Eigen::Vector3d> chain_in_world(threepp::Scene &target, std::string_view named, const named_by_index &segment)
{
    std::vector<Eigen::Vector3d> points;
    for(std::size_t at = 0;; ++at)
    {
        threepp::Object3D *drawn = chain_part(target, named, segment(at));
        if(drawn == nullptr)
            break;

        const std::vector<Eigen::Vector3d> ends = segment_ends(*drawn);
        if(at == 0)
            points.push_back(ends.front());
        points.push_back(ends.back());
    }

    return points;
}

}

#endif
