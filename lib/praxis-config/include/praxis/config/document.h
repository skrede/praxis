#ifndef HPP_GUARD_PRAXIS_CONFIG_DOCUMENT_H
#define HPP_GUARD_PRAXIS_CONFIG_DOCUMENT_H

#include "praxis/config/error.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::config {

namespace detail {
class held_document;
}

enum class origin_kind : std::uint8_t
{
    source,
    fallback,
    instance_required,
    undeclared,
};

// Where one key's value came from. `layer` is the name the source gave itself, and is empty unless
// `kind` is `source`. `instance_required` is a key that crosses a populated collection without an
// ordinal, which is answerable per instance and not from a fallback.
struct value_origin
{
    origin_kind kind;
    std::string layer;
};

// A loaded keyspace: immutable once folded, copyable, and freely readable, so nothing composed from
// one can change what another reader sees. Addressing inside a collection is by ordinal, which is
// what `key` exists to hide; the identity value is stored as data and matched, never addressed.
class document
{
public:
    explicit document(std::shared_ptr<const detail::held_document> held);

    expected<bool, error> flag(std::string_view key) const;

    expected<double, error> real(std::string_view key) const;

    expected<std::string, error> text(std::string_view key) const;

    expected<std::int64_t, error> integer(std::string_view key) const;

    bool holds(std::string_view key) const;

    // The identity values carried by the instances of `collection_path`, in document order.
    std::vector<std::string> identities(std::string_view collection_path) const;

    // The canonical key of `leaf` under the instance whose identity value is `identity`.
    expected<std::string, error> key(std::string_view collection_path, std::string_view identity, std::string_view leaf) const;

    // The kind the declaration named this key's leaf, and nothing where it named none. A key
    // crossing a collection carries an ordinal the declaration never had, which is dropped
    // before the lookup.
    std::optional<field_kind> kind_of(std::string_view key) const;

    value_origin origin_of(std::string_view key) const;

    const std::filesystem::path &source() const noexcept;

private:
    std::shared_ptr<const detail::held_document> m_held;
};

}

#endif
