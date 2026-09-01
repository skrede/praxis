#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_CAPTURED_LOG_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_CAPTURED_LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <string>
#include <sstream>

namespace praxis::tests {

class captured_log
{
public:
    captured_log()
            : previous_(spdlog::default_logger())
    {
        spdlog::set_default_logger(std::make_shared<spdlog::logger>("captured", std::make_shared<spdlog::sinks::ostream_sink_mt>(text_)));
    }

    // The installed sink holds a reference to text_, so a relocated capture would leave it writing
    // into the shell left behind.
    captured_log(const captured_log &)            = delete;
    captured_log(captured_log &&)                 = delete;
    captured_log &operator=(const captured_log &) = delete;
    captured_log &operator=(captured_log &&)      = delete;

    // Installing a default logger also registers it in spdlog's registry under its name, so
    // restoring the previous one displaces it without releasing it and the registry is left holding
    // a sink that writes into text_. Dropping the name is what releases it, and a destructor body
    // runs before any member is destroyed, so text_ outlives the sink either way.
    ~captured_log()
    {
        spdlog::set_default_logger(previous_);
        spdlog::drop("captured");
    }

    std::string text() const
    {
        return text_.str();
    }

private:
    std::ostringstream text_;
    std::shared_ptr<spdlog::logger> previous_;
};

template<typename Command>
std::string reported_by(Command &&command)
{
    captured_log captured;

    command();

    return captured.text();
}

}

#endif
