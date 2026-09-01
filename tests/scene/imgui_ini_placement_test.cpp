#include "imgui_frame.h"

#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <imgui.h>

#include <string>
#include <fstream>
#include <sstream>
#include <utility>
#include <filesystem>
#include <system_error>

namespace {

using praxis::scene::layout_file;

// A folder fixture lives under the system temporary directory only: one left in the repository or
// the build tree is picked up by the convention gate on the next configure, as an unrelated failure.
class scratch_tree
{
public:
    explicit scratch_tree(const std::string &named)
            : m_root(std::filesystem::temp_directory_path() / ("praxis_" + named + "_fixture"))
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
        std::filesystem::create_directories(m_root);
    }

    scratch_tree(const scratch_tree &)            = delete;
    scratch_tree &operator=(const scratch_tree &) = delete;

    ~scratch_tree()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
    }

    const std::filesystem::path &root() const
    {
        return m_root;
    }

private:
    std::filesystem::path m_root;
};

// The working directory is where a layout file lands when nothing points it elsewhere, so a case
// asserting that it did not land there is only meaningful over a directory nothing else writes into.
class started_in
{
public:
    explicit started_in(const std::filesystem::path &where)
            : m_before(std::filesystem::current_path())
    {
        std::filesystem::current_path(where);
    }

    started_in(const started_in &)            = delete;
    started_in &operator=(const started_in &) = delete;

    ~started_in()
    {
        std::error_code ignored;
        std::filesystem::current_path(m_before, ignored);
    }

private:
    std::filesystem::path m_before;
};

class titled_window : public praxis::scene::imgui_window
{
public:
    explicit titled_window(std::string name)
            : imgui_window(std::move(name))
    {
    }

    void render() override
    {
        ImGui::Begin(display_name().c_str());
        ImGui::TextUnformatted("drawn");
        ImGui::End();
    }
};

std::string contents(const std::filesystem::path &of)
{
    std::ifstream read(of, std::ios::binary);
    std::ostringstream whole;
    whole << read.rdbuf();

    return whole.str();
}

}

TEST_CASE("a chosen root carries the layout file and no root leaves the place unchosen", "[scene]")
{
    const std::filesystem::path root = std::filesystem::temp_directory_path() / "praxis_layout_place";

    REQUIRE(layout_file(root).parent_path() == root);
    REQUIRE(layout_file(root).is_absolute());
    REQUIRE(layout_file(std::filesystem::path()).empty());
}

TEST_CASE("a rendered window's layout lands under the chosen root and not where the process started", "[scene]")
{
    const scratch_tree chosen("layout_placement");
    const scratch_tree elsewhere("layout_working_directory");
    const started_in from(elsewhere.root());

    const std::filesystem::path where = layout_file(chosen.root());
    {
        praxis::tests::imgui_frame frame;
        const praxis::scene::placed_layout placed(where);
        titled_window panel("Placement probe");
        frame.draw([&panel] { panel.render(); });
    }

    REQUIRE(std::filesystem::exists(where));
    REQUIRE(std::filesystem::is_empty(elsewhere.root()));
    REQUIRE_THAT(contents(where), Catch::Matchers::ContainsSubstring("[Window][Placement probe]"));
}

// The library reads the path it was handed while the context is torn down, and a context outlives
// every member of a class deriving from it. So the placement must write and unhook before it ends,
// and this drives it in the order the window context destroys it in: placement first, context after.
TEST_CASE("a placement writes the layout out and unhooks the borrowed path before its string dies", "[scene]")
{
    const scratch_tree chosen("layout_unhooked");
    const std::filesystem::path where = layout_file(chosen.root());

    praxis::tests::imgui_frame frame;
    titled_window panel("Unhook probe");
    {
        const praxis::scene::placed_layout placed(where);

        REQUIRE(ImGui::GetIO().IniFilename == placed.where().c_str());
        frame.draw([&panel] { panel.render(); });
    }

    REQUIRE(ImGui::GetIO().IniFilename == nullptr);
    REQUIRE(std::filesystem::exists(where));
    REQUIRE_THAT(contents(where), Catch::Matchers::ContainsSubstring("[Window][Unhook probe]"));
}

TEST_CASE("a placement no frame reached leaves the layout already written where it was", "[scene]")
{
    const scratch_tree chosen("layout_undrawn");
    const std::filesystem::path where = layout_file(chosen.root());
    {
        std::ofstream earlier(where);
        earlier << "[Window][Written before]\nPos=10,10\n";
    }

    praxis::tests::imgui_frame frame;
    {
        const praxis::scene::placed_layout placed(where);
    }

    REQUIRE(ImGui::GetIO().IniFilename == nullptr);
    REQUIRE_THAT(contents(where), Catch::Matchers::ContainsSubstring("[Window][Written before]"));
}

TEST_CASE("an empty place hands the library nothing and leaves its own choice in force", "[scene]")
{
    praxis::tests::imgui_frame frame;
    const char *const before = ImGui::GetIO().IniFilename;
    {
        const praxis::scene::placed_layout placed{std::filesystem::path()};

        REQUIRE(placed.where().empty());
        REQUIRE(ImGui::GetIO().IniFilename == before);
    }

    REQUIRE(ImGui::GetIO().IniFilename == before);
}
