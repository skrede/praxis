#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <utility>
#include <type_traits>

namespace {

class counting_window : public praxis::scene::imgui_window
{
public:
    struct settings
    {
        int count;
    };

    explicit counting_window(std::string name)
            : imgui_window(std::move(name))
            , m_count(0)
    {
    }

    settings state() const
    {
        return settings{m_count};
    }

    void advance()
    {
        ++m_count;
    }

    void render() override
    {
    }

private:
    int m_count;
};

}

TEST_CASE("a window reports the display name it was constructed with", "[scene]")
{
    const counting_window window("Presets");

    REQUIRE(window.display_name() == "Presets");
}

TEST_CASE("two instances of one window class carry distinct display names", "[scene]")
{
    const counting_window first("Left");
    const counting_window second("Right");

    REQUIRE(first.display_name() == "Left");
    REQUIRE(second.display_name() == "Right");
}

TEST_CASE("two instances of one window class keep independent state", "[scene]")
{
    counting_window first("Left");
    counting_window second("Right");

    first.advance();
    first.advance();

    REQUIRE(first.state().count == 2);
    REQUIRE(second.state().count == 0);
}

TEST_CASE("two instances given the same name are both constructible", "[scene]")
{
    const counting_window first("Pose");
    const counting_window second("Pose");

    REQUIRE(first.display_name() == second.display_name());
}

TEST_CASE("the window contract cannot be instantiated on its own", "[scene]")
{
    STATIC_REQUIRE(std::is_abstract_v<praxis::scene::imgui_window>);
    STATIC_REQUIRE(!std::is_constructible_v<praxis::scene::imgui_window, std::string>);
}
