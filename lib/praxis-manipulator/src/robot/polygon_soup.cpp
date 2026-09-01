#include "robot/polygon_soup.h"

#include <Eigen/Core>

#include <cstddef>

namespace praxis::manipulator {

void put_soup_corner(polygon_soup &out, const Eigen::Vector3d &corner)
{
    if(out.written + 3u > out.into.size())
        return;

    out.into[out.written]      = static_cast<float>(corner.x());
    out.into[out.written + 1u] = static_cast<float>(corner.y());
    out.into[out.written + 2u] = static_cast<float>(corner.z());
    out.written += 3u;
}

void clip_polygon(polygon_work &work, const Eigen::Vector3d &out, double reach)
{
    work.kept.clear();
    for(std::size_t at = 0u; at < work.shape.size(); ++at)
    {
        const Eigen::Vector3d here = work.shape[at];
        const Eigen::Vector3d next = work.shape[(at + 1u) % work.shape.size()];
        const double past          = out.dot(here) - reach;
        const double beyond        = out.dot(next) - reach;
        if(past <= 0.0)
            work.kept.push_back(here);
        if((past < 0.0 && beyond > 0.0) || (past > 0.0 && beyond < 0.0))
            work.kept.push_back(here + (next - here) * (past / (past - beyond)));
    }

    work.shape.swap(work.kept);
}

void put_polygon(polygon_soup &out, const polygon_work &work)
{
    for(std::size_t at = 1u; at + 1u < work.shape.size(); ++at)
    {
        put_soup_corner(out, work.shape[0]);
        put_soup_corner(out, work.shape[at]);
        put_soup_corner(out, work.shape[at + 1u]);
    }
}

}
