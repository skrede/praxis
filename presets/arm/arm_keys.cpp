#include "arm_keys.h"

#include "praxis/config/error.h"
#include "praxis/config/document.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::presets::keys {

std::vector<std::string> spelled(std::span<const char *const> labels)
{
    return std::vector<std::string>(labels.begin(), labels.end());
}

std::string text_at(const config::document &values, const std::string &key)
{
    const expected<std::string, config::error> read = values.text(key);

    return read ? read.value() : std::string();
}

double real_at(const config::document &values, const std::string &key)
{
    const expected<double, config::error> read = values.real(key);

    return read ? read.value() : 0.0;
}

std::string keyed(const config::document &values, const std::string &collection, const std::string &identity, std::string_view leaf)
{
    const expected<std::string, config::error> addressed = values.key(collection, identity, leaf);

    return addressed ? addressed.value() : std::string();
}

std::optional<std::size_t> spelled_index(const config::document &values, const std::string &key, std::span<const char *const> labels)
{
    const std::string read = text_at(values, key);
    for(std::size_t option = 0u; option < labels.size(); ++option)
        if(read == labels[option])
            return option;

    return std::nullopt;
}

}
