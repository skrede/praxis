#ifndef HPP_GUARD_PRAXIS_CONFIG_DECLARATION_H
#define HPP_GUARD_PRAXIS_CONFIG_DECLARATION_H

#include <span>
#include <string>
#include <vector>
#include <cstdint>

namespace praxis::config {

enum class field_kind : std::uint8_t
{
    flag,
    integer,
    real,
    text,
    choice,
};

enum class node_kind : std::uint8_t
{
    group,
    collection,
    leaf,
};

// One declared position in a keyspace. `identity` names the leaf whose value keys a collection's
// instances, `fallback` is what a leaf carries when the document omits it, and `allowed` is the
// closed set an enumerated leaf admits.
struct node
{
    node_kind shape;
    field_kind kind;
    std::string path;
    std::string identity;
    std::string fallback;
    std::vector<std::string> allowed;
};

// A keyspace described in the caller's own vocabulary and translated to the engine only where a
// document is loaded against it. Paths are `/`-separated, and a node is declared after the node it
// hangs under.
class declaration
{
public:
    explicit declaration(std::string space);

    declaration &group(std::string path);

    // The instances of `path` are keyed and merged by the leaf named `identity`, which every
    // instance must carry and which must be distinct among them. A collection hangs under a group
    // rather than under the root, because the root carries nothing an identity can be pooled in.
    declaration &collection(std::string path, std::string identity);

    // A typed leaf whose fallback is its value wherever the document supplies none; the document's
    // value is the override, and a read of a key no declaration names is an error rather than this.
    declaration &field(std::string path, field_kind kind, std::string fallback);

    declaration &choice(std::string path, std::vector<std::string> allowed, std::string fallback);

    std::span<const node> nodes() const noexcept;

    const std::string &space() const noexcept;

private:
    std::string m_space;
    std::vector<node> m_nodes;
};

}

#endif
