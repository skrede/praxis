#include "imgui_frame.h"

#include "praxis/scene/labeled_value_window.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <functional>

using namespace praxis;

namespace {

using drawing = std::function<void()>;
using rows    = std::vector<std::vector<scene::labeled_value>>;

const char *const title = "Readout";

std::size_t geometry_of(const drawing &draw)
{
    tests::imgui_frame frames;
    frames.draw(draw);

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

// The panel a case expects, written out here rather than read off the window, so what is compared is
// every vertex the two drawings put on screen.
drawing panel_stating(const std::string &line)
{
    return [line]
    {
        ImGui::Begin(title);
        ImGui::TextUnformatted(line.c_str());
        ImGui::End();
    };
}

// A panel around exactly the calls a case writes out for it, for the readings whose cells are not
// all of one kind and which no helper over a row can stand in for.
drawing panel_around(drawing inner)
{
    return [inner = std::move(inner)]
    {
        ImGui::Begin(title);
        inner();
        ImGui::End();
    };
}

drawing panel_of(const rows &shown, bool beside = true, drawing above = nullptr, drawing below = nullptr)
{
    return [shown, beside, above = std::move(above), below = std::move(below)]
    {
        ImGui::Begin(title);
        if(above)
            above();
        for(const std::vector<scene::labeled_value> &row : shown)
            for(std::size_t entry = 0; entry < row.size(); ++entry)
            {
                if(beside && entry > 0)
                    ImGui::SameLine();
                ImGui::Value(row[entry].label.c_str(), row[entry].value);
            }
        if(below)
            below();
        ImGui::End();
    };
}

std::vector<scene::labeled_value> line(float first, const char *first_label, float second, const char *second_label)
{
    return std::vector<scene::labeled_value>{scene::labeled_value{first, first_label}, scene::labeled_value{second, second_label}};
}

rows one_line_of_two(float first, float second)
{
    return rows{line(first, "X", second, "A")};
}

std::vector<scene::labeled_value> bare(const std::vector<float> &values)
{
    std::vector<scene::labeled_value> built;
    built.reserve(values.size());
    for(float held : values)
        built.push_back(scene::labeled_value{held, std::string()});

    return built;
}

// The same values with a label on every cell, so a labeled and an unlabeled drawing of one reading
// differ in nothing but what the window makes of an empty label.
rows labeled_like(const rows &given)
{
    const char *const marks[4]{"A", "B", "C", "D"};

    rows built = given;
    for(std::vector<scene::labeled_value> &row : built)
        for(std::size_t entry = 0; entry < row.size(); ++entry)
            row[entry].label = marks[entry % 4];

    return built;
}

rows square_from(float first)
{
    rows built;
    for(int row = 0; row < 4; ++row)
    {
        const float base = first + static_cast<float>(row) * 4.f;
        built.push_back(bare({base, base + 1.f, base + 2.f, base + 3.f}));
    }

    return built;
}

// The aligned drawing a case expects, written out here rather than read off the window, exactly as
// the label-value panel above is.
drawing grid_panel_of(const rows &shown, int columns, drawing above = nullptr)
{
    return [shown, columns, above = std::move(above)]
    {
        ImGui::Begin(title);
        if(above)
            above();
        if(ImGui::BeginTable(std::string(title).append("##aligned0").c_str(), columns, ImGuiTableFlags_SizingFixedFit))
        {
            for(const std::vector<scene::labeled_value> &row : shown)
            {
                ImGui::TableNextRow();
                for(const scene::labeled_value &cell : row)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f", cell.value);
                }
            }
            ImGui::EndTable();
        }
        ImGui::End();
    };
}

// What a frame aligned is the column count of every table the context holds once it has been drawn;
// the pool is keyed by identity, so one entry stands for one alignment.
std::vector<int> alignments_of(const drawing &draw)
{
    tests::imgui_frame frames;
    frames.draw(draw);

    std::vector<int> widths;
    ImGuiContext &held = *ImGui::GetCurrentContext();
    for(int slot = 0; slot < held.Tables.GetMapSize(); ++slot)
        if(const ImGuiTable *table = held.Tables.TryGetMapData(slot))
            widths.push_back(table->ColumnsCount);

    return widths;
}

scene::readout_source answering(scene::readout given)
{
    return [given = std::move(given)] { return given; };
}

// The aligned drawing of one cell, written out with the text it is expected to print.
drawing grid_stating(const std::string &printed)
{
    return panel_around(
            [printed]
            {
                if(!ImGui::BeginTable(std::string(title).append("##aligned0").c_str(), 1, ImGuiTableFlags_SizingFixedFit))
                    return;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(printed.c_str());
                ImGui::EndTable();
            });
}

// One value drawn as an aligned cell, as a bare cell beside a label, and as a labeled cell, each
// against a panel written out with the text it is expected to print.
void prints_as(float value, const std::string &printed)
{
    scene::labeled_value_window aligned(title, nullptr, answering(scene::readout{"", rows{bare({value})}}));
    scene::labeled_value_window beside(title, nullptr, answering(scene::readout{"", rows{{scene::labeled_value{1.f, "M"}, scene::labeled_value{value, std::string()}}}}));
    scene::labeled_value_window labeled(title, nullptr, answering(scene::readout{"", rows{{scene::labeled_value{value, "M"}}}}));
    const drawing bare_beside = panel_around(
            [printed]
            {
                ImGui::TextUnformatted("M: 1.000");
                ImGui::SameLine();
                ImGui::TextUnformatted(printed.c_str());
            });

    CHECK(geometry_of([&aligned] { aligned.render(); }) == geometry_of(grid_stating(printed)));
    CHECK(geometry_of([&beside] { beside.render(); }) == geometry_of(bare_beside));
    CHECK(geometry_of([&labeled] { labeled.render(); }) == geometry_of(panel_stating("M: " + printed)));
}

// The interface library identifies a panel by its title, so what a frame opened is the set of titles
// the context holds once it has been drawn; the implicit debug panel is the library's own.
std::vector<std::string> panels_of(const drawing &draw)
{
    tests::imgui_frame frames;
    frames.draw(draw);

    std::vector<std::string> named;
    for(const ImGuiWindow *held : ImGui::GetCurrentContext()->Windows)
        if(held->WasActive && (held->Flags & ImGuiWindowFlags_ChildWindow) == 0 && held->Name != std::string("Debug##Default"))
            named.push_back(held->Name);

    return named;
}

constexpr ImU32 marked_tone = IM_COL32(200, 40, 120, 255);

scene::labeled_value toned(scene::labeled_value cell)
{
    cell.tone = marked_tone;

    return cell;
}

drawing in_tone(drawing call)
{
    return [call = std::move(call)]
    {
        ImGui::PushStyleColor(ImGuiCol_Text, marked_tone);
        call();
        ImGui::PopStyleColor();
    };
}

drawing panel_in_order(drawing before, drawing cell, drawing after)
{
    return panel_around(
            [before = std::move(before), cell = std::move(cell), after = std::move(after)]
            {
                before();
                cell();
                after();
            });
}

// A reading against the panel written out with the toned cell's own call pushed in the tone, and
// against the same panel unpushed, which it must not match.
void draws_in_tone(const rows &shown, const drawing &before, const drawing &cell, const drawing &after)
{
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", shown}));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_in_order(before, in_tone(cell), after)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_in_order(before, cell, after)));
}

const drawing nothing   = [] {};
const drawing into_cell = []
{
    REQUIRE(ImGui::BeginTable(std::string(title).append("##aligned0").c_str(), 1, ImGuiTableFlags_SizingFixedFit));
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
};
const drawing out_of_cell = [] { ImGui::EndTable(); };
const drawing beside_one  = []
{
    ImGui::Value("M", 1.f);
    ImGui::SameLine();
};

}

TEST_CASE("a readout whose source carries a message draws the message and none of the values beside it", "[scene]")
{
    const rows values = one_line_of_two(1.f, 2.f);
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"Nothing to show.", values}));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_stating("Nothing to show.")));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(values)));
}

TEST_CASE("a readout whose source carries rows draws one line per row with its entries side by side", "[scene]")
{
    const rows values{line(1.f, "X", 2.f, "A"), line(3.f, "Y", 4.f, "B"), line(5.f, "Z", 6.f, "C")};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", values}));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(values)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(values, false)));
}

TEST_CASE("the controls a readout is given are drawn once a frame above its values", "[scene]")
{
    const rows values      = one_line_of_two(1.f, 2.f);
    const drawing stated   = [] { ImGui::TextUnformatted("Above"); };
    int invoked            = 0;
    const drawing counting = [&invoked, &stated]
    {
        ++invoked;
        stated();
    };
    scene::labeled_value_window panel(title, counting, answering(scene::readout{"", values}));

    {
        tests::imgui_frame frames;
        frames.draw([&panel] { panel.render(); });
    }
    CHECK(invoked == 2);

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(values, true, stated)));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(values, true, nullptr, stated)));
}

TEST_CASE("a readout given no controls draws its values and nothing above them", "[scene]")
{
    const rows values = one_line_of_two(1.f, 2.f);
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", values}));

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(values)));
}

TEST_CASE("a readout reads its source once a frame and draws what the newest answer carried", "[scene]")
{
    const rows first  = one_line_of_two(1.f, 2.f);
    const rows second = one_line_of_two(3.f, 4.f);
    int reads         = 0;
    scene::labeled_value_window panel(title, nullptr, [&first, &second, &reads] { return scene::readout{"", reads++ == 0 ? first : second}; });

    std::size_t drawn = 0;
    {
        tests::imgui_frame frames;
        frames.draw([&panel] { panel.render(); });
        drawn = frames.signature();
    }

    CHECK(reads == 2);
    CHECK(drawn == geometry_of(panel_of(second)));
    CHECK(drawn != geometry_of(panel_of(first)));
}

TEST_CASE("two readouts given one name draw into one panel", "[scene]")
{
    const rows values = one_line_of_two(1.f, 2.f);
    scene::labeled_value_window first(title, nullptr, answering(scene::readout{"", values}));
    scene::labeled_value_window second(title, nullptr, answering(scene::readout{"", values}));

    CHECK(panels_of(
                  [&first, &second]
                  {
                      first.render();
                      second.render();
                  }) == std::vector<std::string>{title});
}

TEST_CASE("a row whose cells carry no label is drawn as aligned columns rather than as label-value pairs", "[scene]")
{
    const rows unlabeled = rows{bare({1.f, 2.f, 3.f, 4.f})};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", unlabeled}));

    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(grid_panel_of(unlabeled, 4)));
    REQUIRE(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(labeled_like(unlabeled))));
    CHECK(alignments_of([&panel] { panel.render(); }) == std::vector<int>{4});
}

TEST_CASE("four unlabeled rows share one alignment of four columns rather than opening one each", "[scene]")
{
    const rows unlabeled = square_from(1.f);
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", unlabeled}));

    CHECK(alignments_of([&panel] { panel.render(); }) == std::vector<int>{4});
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(grid_panel_of(unlabeled, 4)));
    REQUIRE(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(labeled_like(unlabeled))));
}

TEST_CASE("consecutive unlabeled rows of differing width share one alignment sized to the widest", "[scene]")
{
    const rows unlabeled = rows{bare({1.f, 2.f, 3.f, 4.f}), bare({5.f, 6.f}), bare({7.f, 8.f, 9.f, 10.f})};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", unlabeled}));

    CHECK(alignments_of([&panel] { panel.render(); }) == std::vector<int>{4});
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(grid_panel_of(unlabeled, 4)));
}

TEST_CASE("a labeled row between two unlabeled ones parts them into two alignments", "[scene]")
{
    const rows mixed = rows{bare({1.f, 2.f, 3.f}), line(4.f, "X", 5.f, "A"), bare({6.f, 7.f, 8.f})};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", mixed}));

    const std::vector<int> aligned = alignments_of([&panel] { panel.render(); });
    REQUIRE(aligned.size() == 2u);
    CHECK(aligned[0] == 3);
    CHECK(aligned[1] == 3);
}

TEST_CASE("a labeled row and an unlabeled row below it are drawn each through its own path", "[scene]")
{
    const std::vector<scene::labeled_value> labeled = line(1.f, "X", 2.f, "A");
    const rows unlabeled                            = rows{bare({3.f, 4.f})};
    const drawing above                             = [labeled]
    {
        for(std::size_t entry = 0; entry < labeled.size(); ++entry)
        {
            if(entry > 0)
                ImGui::SameLine();
            ImGui::Value(labeled[entry].label.c_str(), labeled[entry].value);
        }
    };
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", rows{labeled, unlabeled.front()}}));

    CHECK(alignments_of([&panel] { panel.render(); }) == std::vector<int>{2});
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(grid_panel_of(unlabeled, 2, above)));
}

TEST_CASE("a readout carrying a message draws the message and no rows whichever kind of row it holds", "[scene]")
{
    const rows unlabeled = square_from(1.f);
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"Nothing to show.", unlabeled}));

    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_stating("Nothing to show.")));
    CHECK(alignments_of([&panel] { panel.render(); }).empty());
}

TEST_CASE("a readout of no rows and a readout of one unlabeled cell each draw without a reported fault", "[scene]")
{
    scene::labeled_value_window nothing(title, nullptr, answering(scene::readout{}));
    scene::labeled_value_window single(title, nullptr, answering(scene::readout{"", rows{bare({7.f})}}));

    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        frames.draw([&nothing] { nothing.render(); });
        CHECK(frames.has_draw_data());
    }

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&single] { single.render(); });
    CHECK(frames.has_draw_data());
    CHECK(frames.vertices() > 0);
}

TEST_CASE("a reading shaped as the one a pose readout returns is drawn through the label-value path", "[scene]")
{
    const rows pose_shaped{line(1.f, "X", 4.f, "A"), line(2.f, "Y", 5.f, "B"), line(3.f, "Z", 6.f, "C")};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", pose_shaped}));

    CHECK(alignments_of([&panel] { panel.render(); }).empty());
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(panel_of(pose_shaped)));
}

TEST_CASE("a row whose first cell carries a label and whose rest carry none draws the label once and the numbers after it bare", "[scene]")
{
    const rows under_one{std::vector<scene::labeled_value>{scene::labeled_value{1.f, "M"}, scene::labeled_value{2.f, std::string()}, scene::labeled_value{3.f, std::string()}}};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", under_one}));

    const drawing expected = panel_around(
            []
            {
                ImGui::Value("M", 1.f);
                ImGui::SameLine();
                ImGui::Text("%.3f", 2.f);
                ImGui::SameLine();
                ImGui::Text("%.3f", 3.f);
            });

    CHECK(alignments_of([&panel] { panel.render(); }).empty());
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(expected));
    REQUIRE(geometry_of([&panel] { panel.render(); }) != geometry_of(panel_of(labeled_like(under_one))));
}

TEST_CASE("a cell carrying a statement and no label draws the statement alone rather than behind an empty name", "[scene]")
{
    const rows stated{std::vector<scene::labeled_value>{scene::labeled_value{1.f, "M"}, scene::labeled_value{0.f, std::string(), "absent"}}};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", stated}));

    const drawing alone = panel_around(
            []
            {
                ImGui::Value("M", 1.f);
                ImGui::SameLine();
                ImGui::TextUnformatted("absent");
            });
    const drawing behind_a_name = panel_around(
            []
            {
                ImGui::Value("M", 1.f);
                ImGui::SameLine();
                ImGui::Text("%s: %s", "", "absent");
            });

    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(alone));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of(behind_a_name));
}

TEST_CASE("a row whose every cell carries a label draws its number and its statement as it did", "[scene]")
{
    const rows named{std::vector<scene::labeled_value>{scene::labeled_value{1.f, "X"}, scene::labeled_value{0.f, "A", "absent"}}};
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", named}));

    const drawing expected = panel_around(
            []
            {
                ImGui::Value("X", 1.f);
                ImGui::SameLine();
                ImGui::Text("%s: %s", "A", "absent");
            });

    CHECK(alignments_of([&panel] { panel.render(); }).empty());
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(expected));
}

TEST_CASE("an unlabeled run below a row carrying one label is drawn as aligned columns of its own", "[scene]")
{
    const std::vector<scene::labeled_value> under_one{scene::labeled_value{1.f, "M"}, scene::labeled_value{2.f, std::string()}, scene::labeled_value{3.f, std::string()}};
    const rows unlabeled = rows{bare({4.f, 5.f, 6.f}), bare({7.f, 8.f, 9.f})};
    const drawing above  = []
    {
        ImGui::Value("M", 1.f);
        ImGui::SameLine();
        ImGui::Text("%.3f", 2.f);
        ImGui::SameLine();
        ImGui::Text("%.3f", 3.f);
    };
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", rows{under_one, unlabeled[0], unlabeled[1]}}));

    CHECK(alignments_of([&panel] { panel.render(); }) == std::vector<int>{3});
    REQUIRE(geometry_of([&panel] { panel.render(); }) == geometry_of(grid_panel_of(unlabeled, 3, above)));
}

TEST_CASE("a value that prints as zero at three decimals draws unsigned on every drawing path, and every other value draws as it did", "[scene]")
{
    const std::vector<std::pair<float, std::string>> printed{{-0.0f, "0.000"},     {-1e-17f, "0.000"}, {-1e-6f, "0.000"}, {std::nextafter(-0.0005f, 0.0f), "0.000"},
                                                             {-0.0005f, "-0.001"}, {0.0004f, "0.000"}, {-0.25f, "-0.250"}};
    for(const auto &[value, text] : printed)
    {
        INFO("the value " << value << " expected to print as " << text);
        prints_as(value, text);
    }
}

TEST_CASE("a cell carrying a tone draws its text in that tone on every drawing path", "[scene]")
{
    const scene::labeled_value first{1.f, "M"};

    draws_in_tone(rows{{toned({2.f, std::string()})}}, into_cell, [] { ImGui::Text("%.3f", 2.f); }, out_of_cell);
    draws_in_tone(rows{{toned({0.f, std::string(), "absent"})}}, into_cell, [] { ImGui::TextUnformatted("absent"); }, out_of_cell);
    draws_in_tone(rows{{first, toned({2.f, std::string()})}}, beside_one, [] { ImGui::Text("%.3f", 2.f); }, nothing);
    draws_in_tone(rows{{first, toned({0.f, std::string(), "absent"})}}, beside_one, [] { ImGui::TextUnformatted("absent"); }, nothing);
    draws_in_tone(rows{{toned({2.f, "A"})}}, nothing, [] { ImGui::Value("A", 2.f); }, nothing);
    draws_in_tone(rows{{toned({0.f, "A", "absent"})}}, nothing, [] { ImGui::Text("%s: %s", "A", "absent"); }, nothing);
}

TEST_CASE("a tone stops at its own cell, leaving the labeled cell drawn after it in the panel's text color", "[scene]")
{
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", rows{{toned({1.f, "M"}), scene::labeled_value{2.f, "A"}}}}));
    const drawing expected = panel_around(
            []
            {
                in_tone([] { ImGui::Value("M", 1.f); })();
                ImGui::SameLine();
                ImGui::Value("A", 2.f);
            });

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(expected));
}

TEST_CASE("a tone stops at its own cell, leaving the aligned cell drawn after it in the panel's text color", "[scene]")
{
    scene::labeled_value_window panel(title, nullptr, answering(scene::readout{"", rows{{toned({1.f, std::string()}), scene::labeled_value{2.f, std::string()}}}}));
    const drawing expected = panel_around(
            []
            {
                REQUIRE(ImGui::BeginTable(std::string(title).append("##aligned0").c_str(), 2, ImGuiTableFlags_SizingFixedFit));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                in_tone([] { ImGui::Text("%.3f", 1.f); })();
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", 2.f);
                ImGui::EndTable();
            });

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of(expected));
}
