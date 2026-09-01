include_guard(GLOBAL)

include(FetchContent)

# The renderer, the GUI library, the window library and the log library appear in the scene module's
# exported interface as plain target names the generated targets file references but does not
# define. A project consuming that interface includes this file to obtain them; the revisions are
# written here and in no other listfile.

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(spdlog)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
set(THREEPP_USE_EXTERNAL_GLFW OFF)

FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG 2026-08-09
    GIT_SHALLOW TRUE
    SYSTEM
)
FetchContent_MakeAvailable(threepp)

# glfw and the GUI library are built inside threepp's tree; the second is only declared when this
# directory is added.
add_subdirectory("${threepp_SOURCE_DIR}/examples/external" "${threepp_BINARY_DIR}/examples/external" SYSTEM)

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v1.0
    GIT_SHALLOW TRUE
    SYSTEM
)
FetchContent_MakeAvailable(implot)

# The plotting library ships no listfile, so the target carrying its two translation units is
# declared here; the demo unit beside them is not built.
add_library(implot "${implot_SOURCE_DIR}/implot.cpp" "${implot_SOURCE_DIR}/implot_items.cpp")
target_link_libraries(implot PUBLIC imgui)
target_include_directories(implot SYSTEM PUBLIC "${implot_SOURCE_DIR}")
