#include "robot/emit_order.h"

#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

std::vector<int> declaration_order(std::size_t count)
{
    std::vector<int> order(count);
    for(std::size_t i = 0; i < count; ++i)
        order[i] = static_cast<int>(i);

    return order;
}

std::vector<int> subtree_depths(const std::vector<int> &parent_of, const std::vector<int> &order)
{
    std::vector<int> depth(parent_of.size(), 0);
    for(auto it = order.rbegin(); it != order.rend(); ++it)
    {
        const auto child = static_cast<std::size_t>(*it);
        const int parent = parent_of[child];
        if(parent >= 0)
            depth[static_cast<std::size_t>(parent)] = std::max(depth[static_cast<std::size_t>(parent)], depth[child] + 1);
    }

    return depth;
}

std::vector<std::vector<int>> shallowest_first_children(const meios::model<> &model)
{
    const auto &parent_of = model.topo.parent_of;
    const auto depth      = subtree_depths(parent_of, model.topo.order);

    std::vector<std::vector<int>> children(model.links.size());
    for(std::size_t i = 0; i < parent_of.size(); ++i)
        if(parent_of[i] >= 0)
            children[static_cast<std::size_t>(parent_of[i])].push_back(static_cast<int>(i));

    for(auto &siblings : children)
        std::ranges::sort(siblings,
                          [&depth](int lhs, int rhs)
                          {
                              const int lhs_depth = depth[static_cast<std::size_t>(lhs)];
                              const int rhs_depth = depth[static_cast<std::size_t>(rhs)];

                              return lhs_depth != rhs_depth ? lhs_depth < rhs_depth : lhs < rhs;
                          });

    return children;
}

std::vector<int> pre_order(const meios::model<> &model, const std::vector<std::vector<int>> &children)
{
    std::vector<int> order;
    order.reserve(model.links.size());

    std::vector<bool> seen(model.links.size(), false);
    std::vector<int> stack(model.topo.roots.rbegin(), model.topo.roots.rend());
    while(!stack.empty())
    {
        const auto current = static_cast<std::size_t>(stack.back());
        stack.pop_back();
        if(seen[current])
            continue;

        seen[current] = true;
        order.push_back(static_cast<int>(current));

        const auto &siblings = children[current];
        stack.insert(stack.end(), siblings.rbegin(), siblings.rend());
    }

    return order;
}

}

std::vector<int> emit_order(const meios::model<> &model)
{
    if(model.topo.order.empty() || model.topo.parent_of.size() != model.links.size())
        return declaration_order(model.links.size());

    return pre_order(model, shallowest_first_children(model));
}

}
