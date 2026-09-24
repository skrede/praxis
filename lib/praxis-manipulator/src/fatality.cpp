#include "fatality.h"

#include <spdlog/spdlog.h>

#include <string>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

namespace {

bool is_fatal(refusal reason)
{
    return reason == refusal::unsupported_input || reason == refusal::degenerate;
}

}

void tear_down_if_fatal(std::string_view named, refusal reason, refusal_standing standing, request_origin formed_from, const std::function<void(std::string)> &ask_unload)
{
    if(!is_fatal(reason) || standing == refusal_standing::composition_wide || formed_from == request_origin::edited || ask_unload == nullptr)
        return;

    spdlog::error("praxis: '{}' refused the request, so the composition cannot answer for itself and is asked to unload", named);
    ask_unload(std::string(named));
}

}
