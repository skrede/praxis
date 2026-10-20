#include "surface_mesh.h"
#include "scenario_defaults.h"

#include "praxis/optimization/surface_stencil.h"

#include <threepp/math/MathUtils.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <threepp/geometries/SphereGeometry.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace praxis::optimization {

namespace {

std::shared_ptr<threepp::Material> lit(std::uint32_t tone, bool both_sides)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);
    if(both_sides)
        material->side = threepp::Side::Double;

    return material;
}

bool within(const std::vector<interval> &ranges, const Eigen::VectorXd &x)
{
    return x.size() == 2 && x[0] >= ranges[0].lower && x[0] <= ranges[0].upper && x[1] >= ranges[1].lower && x[1] <= ranges[1].upper;
}

}

// The scene's frame is y-up and the surface is written z-up, so everything hangs under one root
// carrying that quarter turn.
surface_stencil::surface_stencil(threepp::Scene &parent, scenario_reader seen, std::shared_ptr<const surface_view> drawn)
        : m_scene(parent)
        , m_seen(std::move(seen))
        , m_view(std::move(drawn))
        , m_root(threepp::Group::create())
        , m_field(threepp::Mesh::create(field_geometry(surface_samples), lit(surface_tone, true)))
        , m_iterates(threepp::Group::create())
        , m_iterate_tone(lit(iterate_tone, false))
        , m_selected_tone(lit(selected_iterate_tone, false))
        , m_sphere(threepp::SphereGeometry::create(static_cast<float>(iterate_radius)))
        , m_sampled()
        , m_markers()
{
    m_root->rotation.x = -threepp::math::PI / 2.f;
    m_field->name      = "surface";
    m_iterates->name   = "iterates";
    m_root->add(m_field);
    m_root->add(m_iterates);
}

expected<void, refusal> surface_stencil::initialize()
{
    m_scene.add(m_root);

    return {};
}

void surface_stencil::tear_down()
{
    m_scene.remove(*m_root);
}

void surface_stencil::render() const
{
    m_field->visible    = m_view == nullptr || m_view->field;
    m_iterates->visible = m_view == nullptr || m_view->iterates;

    const std::shared_ptr<const scenario_snapshot> shown = m_seen.read();
    if(shown == nullptr || shown->grid == nullptr || shown->ranges.size() != 2u)
        return;

    if(shown->grid != m_sampled)
    {
        write_field(*m_field->geometry(), *shown->grid, shown->ranges, placement_of(shown->ranges, *shown->grid));
        m_sampled = shown->grid;
    }
    place_iterates(*shown);
}

void surface_stencil::place_iterates(const scenario_snapshot &shown) const
{
    const field_placement placed = placement_of(shown.ranges, *shown.grid);
    const std::size_t count      = shown.run.iterates.size();
    while(m_markers.size() < count)
    {
        m_markers.push_back(threepp::Mesh::create(m_sphere, m_iterate_tone));
        m_markers.back()->name = "iterate";
        m_iterates->add(m_markers.back());
    }

    for(std::size_t k = 0; k < m_markers.size(); ++k)
    {
        threepp::Mesh &marker = *m_markers[k];
        marker.visible        = k < count && within(shown.ranges, shown.run.iterates[k]) && k < shown.run.errors.size() && shown.run.errors[k].has_value();
        if(!marker.visible)
            continue;

        const Eigen::Vector3d at = placed.at(shown.run.iterates[k][0], shown.run.iterates[k][1], -(*shown.run.errors[k])[0]);
        marker.position.set(static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z()));
        marker.setMaterial(k == shown.step ? m_selected_tone : m_iterate_tone);
    }
}

}
