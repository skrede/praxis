#include "model_chain.h"
#include "model_topology.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <cstddef>
#include <iterator>
#include <algorithm>

namespace praxis::manipulator {

namespace {

struct chain_metrics
{
    int depth;
    int actuated;
};

int link_by_name(const meios::model<> &model, const std::string &name)
{
    const auto it = std::ranges::find_if(model.links, [&name](const auto &link) { return link.name == name; });

    return it == model.links.end() ? -1 : static_cast<int>(std::distance(model.links.begin(), it));
}

chain_metrics measure_chain(const meios::model<> &model, int link, int root)
{
    chain_metrics metrics{0, 0};
    for(int current = link; current != root; current = model.topo.parent_of[static_cast<std::size_t>(current)])
    {
        if(current < 0 || metrics.depth > static_cast<int>(model.links.size()))
            return {-1, -1};

        const meios::joint<> *joint = joint_above(model, current);
        if(joint && is_actuated(joint->kind))
            ++metrics.actuated;
        ++metrics.depth;
    }

    return metrics;
}

std::vector<bool> childless(const meios::model<> &model)
{
    std::vector<bool> leaf(model.links.size(), true);
    for(const int parent : model.topo.parent_of)
        if(parent >= 0 && static_cast<std::size_t>(parent) < leaf.size())
            leaf[static_cast<std::size_t>(parent)] = false;

    return leaf;
}

bool deeper(chain_metrics lhs, chain_metrics rhs)
{
    return lhs.actuated > rhs.actuated || (lhs.actuated == rhs.actuated && lhs.depth > rhs.depth);
}

int deepest_leaf(const meios::model<> &model, int root, std::vector<std::string> &ties)
{
    const auto leaf = childless(model);

    int tip = -1;
    chain_metrics best{-1, -1};
    for(const int index : model.topo.order)
    {
        const auto slot    = static_cast<std::size_t>(index);
        const auto metrics = leaf[slot] ? measure_chain(model, index, root) : chain_metrics{-1, -1};
        if(metrics.actuated < 0)
            continue;

        if(deeper(metrics, best))
        {
            tip  = index;
            best = metrics;
            ties.assign(1, model.links[slot].name);
        }
        else if(metrics.actuated == best.actuated && metrics.depth == best.depth)
            ties.push_back(model.links[slot].name);
    }

    return tip;
}

void warn_ambiguous_tip(const std::vector<std::string> &ties, const std::string &tip)
{
    std::string candidates = ties.front();
    for(std::size_t i = 1; i < ties.size(); ++i)
        candidates += ", " + ties[i];

    spdlog::warn("praxis: leaf links {} are equally deep; taking '{}' as the tip -- name one to choose", candidates, tip);
}

expected<int, refusal> select_tip(const meios::model<> &model, int root)
{
    std::vector<std::string> ties;
    const int tip = deepest_leaf(model, root, ties);
    if(tip < 0)
    {
        spdlog::warn("praxis: model '{}' has no leaf link reachable from its root", model.name);
        return unexpected(refusal::unsupported_input);
    }

    if(ties.size() > 1)
        warn_ambiguous_tip(ties, model.links[static_cast<std::size_t>(tip)].name);

    return tip;
}

expected<int, refusal> chain_tip(const meios::model<> &model, const std::string &tip_link)
{
    const auto root = root_link(model);
    if(!root)
        return unexpected(root.error());
    if(tip_link.empty())
        return select_tip(model, *root);

    const int tip = link_by_name(model, tip_link);
    if(tip < 0)
    {
        spdlog::warn("praxis: model '{}' has no link named '{}'", model.name, tip_link);
        return unexpected(refusal::unsupported_input);
    }

    return tip;
}

expected<std::vector<int>, refusal> walk_to_root(const meios::model<> &model, int tip, const std::string &leaf)
{
    std::vector<int> chain;
    for(int current = tip; current >= 0; current = model.topo.parent_of[static_cast<std::size_t>(current)])
    {
        chain.push_back(current);
        if(chain.size() > model.links.size())
        {
            spdlog::warn("praxis: the parent chain of link '{}' is cyclic", leaf);
            return unexpected(refusal::degenerate);
        }
    }
    std::ranges::reverse(chain);

    return chain;
}

}

bool is_actuated(meios::joint_kind kind)
{
    return kind == meios::joint_kind::revolute || kind == meios::joint_kind::continuous || kind == meios::joint_kind::prismatic;
}

const meios::joint<> *joint_above(const meios::model<> &model, int link)
{
    if(link < 0 || static_cast<std::size_t>(link) >= model.topo.joint_of.size())
        return nullptr;

    const int index = model.topo.joint_of[static_cast<std::size_t>(link)];

    return index >= 0 && static_cast<std::size_t>(index) < model.joints.size() ? &model.joints[static_cast<std::size_t>(index)] : nullptr;
}

expected<int, refusal> root_link(const meios::model<> &model)
{
    if(const auto addressable = addressable_topology(model); !addressable)
        return unexpected(addressable.error());

    return model.topo.roots.front();
}

expected<std::vector<int>, refusal> link_chain(const meios::model<> &model, const std::string &tip_link)
{
    const auto tip = chain_tip(model, tip_link);
    if(!tip)
        return unexpected(tip.error());

    const std::string &leaf = model.links[static_cast<std::size_t>(*tip)].name;
    auto chain              = walk_to_root(model, *tip, leaf);
    if(!chain)
        return unexpected(chain.error());

    const int root = model.topo.roots.front();
    if(chain->front() != root)
    {
        spdlog::warn("praxis: link '{}' is not connected to root link '{}'", leaf, model.links[static_cast<std::size_t>(root)].name);
        return unexpected(refusal::degenerate);
    }

    return chain;
}

}
