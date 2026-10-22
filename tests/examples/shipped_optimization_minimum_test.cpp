#include "scratch_documents.h"
#include "offered_optimization.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/settings_keys.h"

#include "praxis/config/store.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace scratch_documents;
using namespace offered_optimization;

TEST_CASE("the shipped minimum-form documents state the minimum form and Newton's method", "[examples][optimization]")
{
    for(const char *document : {"curve-minimum.xml", "surface-minimum.xml"})
    {
        INFO(document);
        const config::location at  = config::resolve(document, PRAXIS_SHIPPED_OPTIMIZATION_DIR);
        const config::outcome read = config::load_or_defaults(presets::optimization_keyspace(), at, config::expectation::partial);
        REQUIRE(std::filesystem::exists(at.resolved));
        REQUIRE_FALSE(read.failure.has_value());
        CHECK(read.values.text(optimization::form_key).value_or(std::string()) == "minimum");
        CHECK(read.values.text(optimization::method_key).value_or(std::string()) == "newton");
        CHECK(read.values.origin_of(optimization::step_length_key).kind == config::origin_kind::source);
    }
}

TEST_CASE("each shipped minimum-form document publishes a converged run from its seed through the demonstration's registration", "[examples][optimization]")
{
    const scratch where("shipped-optimization-minimum");
    const offered shipped(where);

    for(const std::string named : {"Minimization: x^4 - 3x^2 + x", "Minimization: (x^2 + 10y^2)/2"})
    {
        INFO(named);
        REQUIRE(std::ranges::find(shipped.names, named) != shipped.names.end());

        const std::string drawn = solve_text(*shipped.registry, named);
        CHECK(drawn.find("Converged. Steps taken: ") != std::string::npos);
        CHECK(drawn.find("Converged. Steps taken: 0.") == std::string::npos);
    }
}
