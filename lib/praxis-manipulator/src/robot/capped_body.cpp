#include "robot/unit_mesh.h"
#include "robot/capped_body.h"
#include "robot/polygon_soup.h"

#include <Eigen/Core>

#include <span>
#include <array>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// The box the surface is cut against, and which of its three axes cut at all.
struct cut_box
{
    Eigen::Vector3d half;
    std::array<bool, 3> cuts;
};

// Where a triangle stands against the box: wholly within it, wholly past one of its planes, or
// across a plane and so in need of clipping.
enum class standing
{
    within,
    beyond,
    across
};

bool cut_along(const cut_box &box, Eigen::Index axis)
{
    return box.cuts[static_cast<std::size_t>(axis)];
}

cut_box box_of(const Eigen::Vector3d &half_widths)
{
    cut_box box{half_widths, {false, false, false}};
    for(std::size_t axis = 0u; axis < 3u; ++axis)
        box.cuts[axis] = half_widths[static_cast<Eigen::Index>(axis)] < unit_mesh_inradius();

    return box;
}

Eigen::Vector3d along_axis(Eigen::Index axis, double reach)
{
    Eigen::Vector3d out = Eigen::Vector3d::Zero();
    out[axis]           = reach;

    return out;
}

standing where_it_stands(const mesh_facet &facet, const cut_box &box)
{
    standing put = standing::within;
    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        if(!cut_along(box, axis))
            continue;

        if(facet.least[axis] > box.half[axis] || facet.most[axis] < -box.half[axis])
            return standing::beyond;

        if(facet.most[axis] > box.half[axis] || facet.least[axis] < -box.half[axis])
            put = standing::across;
    }

    return put;
}

void put_clipped_triangle(polygon_soup &out, polygon_work &work, const cut_box &box, std::size_t at)
{
    const std::span<const Eigen::Vector3d> corners = unit_mesh_corners();
    work.shape.assign(corners.data() + at, corners.data() + at + 3u);
    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        if(!cut_along(box, axis))
            continue;

        for(const double sign : {1.0, -1.0})
        {
            clip_polygon(work, along_axis(axis, sign), box.half[axis]);
            if(work.shape.size() < 3u)
                return;
        }
    }

    put_polygon(out, work);
}

void put_surface(polygon_soup &out, polygon_work &work, const cut_box &box)
{
    const std::span<const Eigen::Vector3d> corners = unit_mesh_corners();
    const std::span<const mesh_facet> facets       = unit_mesh_facets();
    for(std::size_t at = 0u; at < facets.size(); ++at)
    {
        const standing put = where_it_stands(facets[at], box);
        if(put == standing::beyond)
            continue;

        if(put == standing::across)
            put_clipped_triangle(out, work, box, 3u * at);
        else
            for(std::size_t corner = 0u; corner < 3u; ++corner)
                put_soup_corner(out, corners[3u * at + corner]);
    }
}

// The rectangle one plane of the box cuts out of it, wound so that its normal points away from the
// origin. An axis that is not cut bounds the rectangle at one, which is past every corner of a mesh
// inscribed in the unit sphere.
void face_rectangle(polygon_work &work, const cut_box &box, Eigen::Index axis, double sign)
{
    Eigen::Vector3d reach = Eigen::Vector3d::Ones();
    for(Eigen::Index other = 0; other < 3; ++other)
        if(cut_along(box, other))
            reach[other] = box.half[other];
    reach[axis] = 0.0;

    const Eigen::Index first  = (axis + 1) % 3;
    const Eigen::Index second = (axis + 2) % 3;
    const Eigen::Vector3d hub = along_axis(axis, sign * box.half[axis]);
    const Eigen::Vector3d one = along_axis(first, reach[first]);
    const Eigen::Vector3d two = along_axis(second, reach[second]);

    work.shape.assign({hub - one - two, hub + one - two, hub + one + two, hub - one + two});
    if(sign < 0.0)
        std::reverse(work.shape.begin(), work.shape.end());
}

// A flat face is the rectangle of its own plane trimmed by every triangle of the mesh that reaches
// across that plane. Those are the only triangles that can trim it: the body is convex and holds
// the plane's own axis point, so the rectangle leaves the mesh through a triangle meeting the plane.
void put_face(polygon_soup &out, polygon_work &work, const cut_box &box, Eigen::Index axis, double sign)
{
    face_rectangle(work, box, axis, sign);

    const double stands = sign * box.half[axis];
    for(const mesh_facet &facet : unit_mesh_facets())
    {
        if(facet.least[axis] > stands || facet.most[axis] < stands)
            continue;

        clip_polygon(work, facet.out, facet.reach);
        if(work.shape.size() < 3u)
            return;
    }

    put_polygon(out, work);
}

}

std::size_t capped_body_vertex_bound()
{
    return 3u * (13u * unit_mesh_facets().size() + 12u);
}

std::size_t cap_unit_body(const Eigen::Vector3d &half_widths, std::span<float> into)
{
    const cut_box box = box_of(half_widths);
    polygon_soup out{into, 0u};
    polygon_work work;

    put_surface(out, work, box);
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        if(cut_along(box, axis))
            for(const double sign : {1.0, -1.0})
                put_face(out, work, box, axis, sign);

    return out.written / 3u;
}

}
