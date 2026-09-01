#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_IMGUI_FRAME_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_IMGUI_FRAME_H

#include "praxis/scene/imgui_window.h"

#include <imgui.h>

#include <string>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::tests {

// One context with no renderer attached, for its lifetime. The context is global state the library
// reaches through a single pointer, so a relocated fixture would leave a live context behind.
class imgui_frame
{
public:
    imgui_frame()
            : lists_(0)
            , valid_(false)
            , vertices_(0)
            , signature_(0u)
            , ini_()
            , plots_()
    {
        open();
    }

    explicit imgui_frame(const std::filesystem::path &ini)
            : lists_(0)
            , valid_(false)
            , vertices_(0)
            , signature_(0u)
            , ini_(ini.string())
            , plots_()
    {
        open();
    }

    imgui_frame(const imgui_frame &)            = delete;
    imgui_frame(imgui_frame &&)                 = delete;
    imgui_frame &operator=(const imgui_frame &) = delete;
    imgui_frame &operator=(imgui_frame &&)      = delete;

    ~imgui_frame()
    {
        plots_.reset();
        ImGui::DestroyContext();
    }

    template<typename drawing>
    void draw_frame(drawing &&contents)
    {
        ImGui::NewFrame();
        std::forward<drawing>(contents)();
        ImGui::Render();

        const ImDrawData *drawn = ImGui::GetDrawData();
        valid_                  = drawn != nullptr && drawn->Valid;
        lists_                  = drawn != nullptr ? drawn->CmdListsCount : 0;
        vertices_               = drawn != nullptr ? drawn->TotalVtxCount : 0;
        signature_              = drawn != nullptr ? hashed(*drawn) : 0u;
    }

    // A window emits no geometry on the frame it first appears on, so a single frame is not enough
    // to see that one drew.
    template<typename drawing>
    void draw(drawing &&contents, int frames = 2)
    {
        for(int frame = 0; frame < frames; ++frame)
            draw_frame(contents);
    }

    bool has_draw_data() const
    {
        return valid_;
    }

    int command_lists() const
    {
        return lists_;
    }

    int vertices() const
    {
        return vertices_;
    }

    // Every vertex drawn, position and atlas coordinate alike, so two panels differing only in
    // which glyphs they put on screen answer differently. A vertex count cannot: a visible glyph
    // costs four vertices whichever glyph it is.
    std::size_t signature() const
    {
        return signature_;
    }

    void assert_on_frame_faults(bool asserting)
    {
        ImGui::GetIO().ConfigErrorRecoveryEnableAssert = asserting;
    }

private:
    int lists_;
    bool valid_;
    int vertices_;
    std::size_t signature_;
    std::string ini_;
    std::optional<praxis::scene::plot_context> plots_;

    static std::size_t hashed(const ImDrawData &drawn)
    {
        std::size_t mixed = 0xcbf29ce484222325ULL;
        for(int list = 0; list < drawn.CmdListsCount; ++list)
        {
            const ImDrawList &commands = *drawn.CmdLists[list];
            const auto *at             = reinterpret_cast<const unsigned char *>(commands.VtxBuffer.Data);
            for(std::size_t byte = 0u; byte < static_cast<std::size_t>(commands.VtxBuffer.Size) * sizeof(ImDrawVert); ++byte)
                mixed = (mixed ^ at[byte]) * 0x100000001b3ULL;
        }

        return mixed;
    }

    void open()
    {
        ImGui::CreateContext();

        ImGuiIO &io    = ImGui::GetIO();
        io.DisplaySize = ImVec2(1920.f, 1080.f);
        io.DeltaTime   = 1.f / 60.f;
        io.IniFilename = ini_.empty() ? nullptr : ini_.c_str();

        // A backend reporting texture support builds the atlas; with none attached the first frame
        // asserts inside the font update unless it is built here.
        unsigned char *pixels = nullptr;
        int width             = 0;
        int height            = 0;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

        plots_.emplace();
    }
};

}

#endif
