#include "praxis/compat/detail/decimal.h"

#include <cmath>
#include <cerrno>
#include <string>
#include <cstdlib>
#include <charconv>
#include <optional>
#include <string_view>

#include <locale.h>
// macOS declares newlocale and strtod_l here, for the headers included before it.
#if __has_include(<xlocale.h>)
    #include <xlocale.h>
#endif

namespace praxis::detail {

#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
std::optional<double> decimal_value(std::string_view text)
{
    double value                      = 0.0;
    const std::from_chars_result done = std::from_chars(text.data(), text.data() + text.size(), value);
    if(done.ec != std::errc() || done.ptr != text.data() + text.size())
        return std::nullopt;
    return value;
}
#else
std::optional<double> decimal_value(std::string_view text)
{
    static const locale_t classic = newlocale(LC_ALL_MASK, "C", locale_t{});
    if(classic == locale_t{})
        return std::nullopt;

    const std::string held(text);
    char *end          = nullptr;
    errno              = 0;
    const double value = strtod_l(held.c_str(), &end, classic);
    if(end != held.c_str() + held.size() || (errno == ERANGE && (value == 0.0 || std::isinf(value))))
        return std::nullopt;
    return value;
}
#endif

}
