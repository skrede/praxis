#include "robot/chain_figure.h"
#include "robot/chain_placement.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <threepp/objects/Group.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>

#include <span>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr threepp::Color::ColorName solution_tone = threepp::Color::limegreen;

refusal narrower_or_wider_than(std::size_t joints, Eigen::Index width)
{
    spdlog::error("praxis: the drawing carries {} joints and a configuration it was told to stand at names {}, so no figure is drawn and the ones standing are left as they were",
                  joints, width);

    return refusal::unsupported_input;
}

}

std::shared_ptr<threepp::Material> solution_material()
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", threepp::Color(solution_tone)}});
}

std::string loadable_robot_stencil::solution_figure_name(std::size_t figure)
{
    return "Solution figure " + std::to_string(figure + 1);
}

std::string loadable_robot_stencil::solution_segment_name(std::size_t figure, std::size_t segment)
{
    return solution_figure_name(figure) + " segment " + std::to_string(segment + 1);
}

std::string loadable_robot_stencil::solution_mark_name(std::size_t figure, std::size_t joint)
{
    return solution_figure_name(figure) + " origin mark " + std::to_string(joint + 1);
}

// Every configuration is folded before anything standing is removed, so a set carrying one the
// stencil cannot draw leaves the figures already standing untouched.
expected<void, refusal> loadable_robot_stencil::set_solution_figures(std::span<const joint_vector> at)
{
    const expected<std::vector<std::vector<Eigen::Vector3d>>, refusal> folded = fold_solutions(at);
    if(!folded)
        return unexpected(folded.error());

    clear_solution_figures();
    for(std::size_t figure = 0; figure < folded->size(); ++figure)
    {
        raise_solution_figure(figure, m_screws.size());
        place_chain_figure(m_solution_figures[figure].segments, m_solution_figures[figure].marks, (*folded)[figure]);
    }

    return {};
}

void loadable_robot_stencil::clear_solution_figures()
{
    for(const drawn_figure &standing : m_solution_figures)
        m_solutions->remove(*standing.object);

    m_solution_figures.clear();
}

void loadable_robot_stencil::set_solution_figures_shown(bool shown)
{
    m_solutions->visible = shown;
}

expected<std::vector<std::vector<Eigen::Vector3d>>, refusal> loadable_robot_stencil::fold_solutions(std::span<const joint_vector> at) const
{
    std::vector<std::vector<Eigen::Vector3d>> folded;
    folded.reserve(at.size());
    for(const joint_vector &configuration : at)
    {
        if(configuration.size() != static_cast<Eigen::Index>(m_screws.size()))
            return unexpected(narrower_or_wider_than(m_screws.size(), configuration.size()));

        const expected<std::vector<Eigen::Vector3d>, refusal> points = fold_joint_origins(m_home, m_screws, configuration, m_screw);
        if(!points)
        {
            spdlog::error(
                    "praxis: a configuration the drawing was told to stand at cannot be folded to joint origins, so no figure is drawn and the ones standing are left as they were");

            return unexpected(points.error());
        }

        folded.push_back(*points);
    }

    return folded;
}

// The same objects the arm's own chain is built from, at the same count: one segment per gap between
// the folded points and one mark per joint origin.
void loadable_robot_stencil::raise_solution_figure(std::size_t figure, std::size_t joints)
{
    const std::size_t segments = joints == 0u ? 0u : joints + 1u;

    drawn_figure &raised = m_solution_figures.emplace_back();
    raised.object        = threepp::Group::create();
    raised.object->name  = solution_figure_name(figure);
    for(std::size_t at = 0; at < segments; ++at)
        raised.object->add(raised.segments.emplace_back(chain_segment_object(solution_segment_name(figure, at), m_solution_tone)));
    for(std::size_t joint = 0; joint < joints; ++joint)
        raised.object->add(raised.marks.emplace_back(joint_mark_object(solution_mark_name(figure, joint), m_solution_tone)));

    m_solutions->add(raised.object);
}

}
