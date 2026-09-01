#include "model_topology.h"

#include <spdlog/spdlog.h>

#include <vector>
#include <cstddef>
#include <algorithm>
#include <string_view>

namespace praxis::manipulator {

namespace {

// Absence is the sentinel and nothing else: the readers downstream test the sign of what they read,
// so admitting any negative entry here would leave one of them indexing on a value another took for
// an absence.
bool addresses_within(const std::vector<int> &table, std::size_t bound, bool absence_allowed)
{
    const auto addresses = [bound, absence_allowed](const int entry) { return entry < 0 ? absence_allowed && entry == -1 : static_cast<std::size_t>(entry) < bound; };

    return std::ranges::all_of(table, addresses);
}

expected<void, refusal> unaddressable(const meios::model<> &model, std::string_view table)
{
    spdlog::error("praxis: model '{}' has a {} entry that addresses no link", model.name, table);

    return unexpected(refusal::degenerate);
}

expected<void, refusal> populated(const meios::model<> &model)
{
    if(model.links.empty())
    {
        spdlog::warn("praxis: model '{}' has no links", model.name);
        return unexpected(refusal::unsupported_input);
    }
    if(model.topo.roots.empty())
    {
        spdlog::warn("praxis: model '{}' has no root link", model.name);
        return unexpected(refusal::unsupported_input);
    }

    return {};
}

// The joint table is bounded by joint_above, which every reader of it goes through.
expected<void, refusal> tables_address(const meios::model<> &model)
{
    const std::size_t links = model.links.size();
    if(model.topo.parent_of.size() != links)
    {
        spdlog::error("praxis: model '{}' has {} links but {} parent entries", model.name, links, model.topo.parent_of.size());
        return unexpected(refusal::degenerate);
    }

    if(!addresses_within(model.topo.parent_of, links, true))
        return unaddressable(model, "parent");
    if(!addresses_within(model.topo.order, links, false))
        return unaddressable(model, "traversal order");
    if(!addresses_within(model.topo.roots, links, false))
        return unaddressable(model, "root");

    return {};
}

// A chain that has taken more steps than there are links has revisited one, which is the bound the
// chain walk uses for its own cycle check.
bool reaches_absence(const std::vector<int> &parent_of, int link, std::size_t bound)
{
    int current = link;
    for(std::size_t steps = 0; steps <= bound; ++steps)
    {
        if(current < 0)
            return true;
        current = parent_of[static_cast<std::size_t>(current)];
    }

    return false;
}

expected<void, refusal> parents_terminate(const meios::model<> &model)
{
    const auto &parent_of = model.topo.parent_of;
    for(const int root : model.topo.roots)
        if(parent_of[static_cast<std::size_t>(root)] >= 0)
        {
            spdlog::error("praxis: model '{}' names link '{}' a root and also gives it a parent", model.name, model.links[static_cast<std::size_t>(root)].name);
            return unexpected(refusal::degenerate);
        }

    for(std::size_t link = 0; link < parent_of.size(); ++link)
        if(!reaches_absence(parent_of, static_cast<int>(link), parent_of.size()))
        {
            spdlog::error("praxis: in model '{}' the parent chain of link '{}' is cyclic", model.name, model.links[link].name);
            return unexpected(refusal::degenerate);
        }

    return {};
}

}

expected<void, refusal> addressable_topology(const meios::model<> &model)
{
    if(const auto present = populated(model); !present)
        return unexpected(present.error());

    // Walkability indexes the parent and root tables, so it runs only once tables_address has
    // bounded both.
    if(const auto addressed = tables_address(model); !addressed)
        return unexpected(addressed.error());

    return parents_terminate(model);
}

}
