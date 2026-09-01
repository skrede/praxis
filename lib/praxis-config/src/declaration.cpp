#include "praxis/config/declaration.h"

#include <span>
#include <string>
#include <vector>
#include <utility>

namespace praxis::config {

declaration::declaration(std::string space)
        : m_space(std::move(space))
        , m_nodes()
{
}

declaration &declaration::group(std::string path)
{
    m_nodes.push_back(node{node_kind::group, field_kind::text, std::move(path), {}, {}, {}});
    return *this;
}

declaration &declaration::collection(std::string path, std::string identity)
{
    m_nodes.push_back(node{node_kind::collection, field_kind::text, std::move(path), std::move(identity), {}, {}});
    return *this;
}

declaration &declaration::field(std::string path, field_kind kind, std::string fallback)
{
    m_nodes.push_back(node{node_kind::leaf, kind, std::move(path), {}, std::move(fallback), {}});
    return *this;
}

declaration &declaration::choice(std::string path, std::vector<std::string> allowed, std::string fallback)
{
    m_nodes.push_back(node{node_kind::leaf, field_kind::choice, std::move(path), {}, std::move(fallback), std::move(allowed)});
    return *this;
}

std::span<const node> declaration::nodes() const noexcept
{
    return m_nodes;
}

const std::string &declaration::space() const noexcept
{
    return m_space;
}

}
