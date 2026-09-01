#include "scratch_documents.h"

#include "demo_documents.h"

#include "praxis/config/store.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <filesystem>

using namespace praxis;
using namespace scratch_documents;

TEST_CASE("A document with no copy of its own is read from the seed", "[examples][documents]")
{
    const scratch where("reading-the-seed");
    author(where.seeds() / "machine.xml", authored);

    const demo::documents mine(where.seeds(), where.state());

    REQUIRE(mine.reading("machine.xml").resolved == std::filesystem::weakly_canonical(where.seeds() / "machine.xml"));
    REQUIRE(mine.seeds() == where.seeds());
}

TEST_CASE("A document with a copy of its own is read from the copy", "[examples][documents]")
{
    const scratch where("reading-the-copy");
    author(where.seeds() / "machine.xml", authored);
    author(where.state() / "machine.xml", already_there);

    const demo::documents mine(where.seeds(), where.state());

    REQUIRE(mine.reading("machine.xml").resolved == std::filesystem::weakly_canonical(where.state() / "machine.xml"));
}

TEST_CASE("A write makes the state directory and answers the copy", "[examples][documents]")
{
    const scratch where("making-the-directory");
    author(where.seeds() / "machine.xml", authored);

    const demo::documents mine(where.seeds(), where.state());
    REQUIRE_FALSE(std::filesystem::exists(where.state()));

    const config::location target = mine.writing("machine.xml");

    REQUIRE(std::filesystem::is_directory(where.state()));
    REQUIRE(target.resolved == std::filesystem::weakly_canonical(where.state() / "machine.xml"));
}

TEST_CASE("A first write reproduces the seed byte for byte", "[examples][documents]")
{
    const scratch where("reproducing-the-seed");
    author(where.seeds() / "machine.xml", authored);

    const demo::documents mine(where.seeds(), where.state());
    const config::location target = mine.writing("machine.xml");

    REQUIRE(std::filesystem::exists(target.resolved));
    REQUIRE(bytes_of(target.resolved) == std::string(authored));
    REQUIRE(bytes_of(target.resolved).find("<!-- the line an author wrote -->") != std::string::npos);
}

TEST_CASE("A write against a copy that is there leaves its bytes alone", "[examples][documents]")
{
    const scratch where("keeping-the-copy");
    author(where.seeds() / "machine.xml", authored);
    author(where.state() / "machine.xml", already_there);

    const demo::documents mine(where.seeds(), where.state());
    const config::location target = mine.writing("machine.xml");

    REQUIRE(bytes_of(target.resolved) == std::string(already_there));
}

TEST_CASE("A name no seed carries is answered without a file being made", "[examples][documents]")
{
    const scratch where("no-seed-at-all");

    const demo::documents mine(where.seeds(), where.state());
    const config::location target = mine.writing("supplied-chain.xml");

    REQUIRE(target.resolved == std::filesystem::weakly_canonical(where.state() / "supplied-chain.xml"));
    REQUIRE_FALSE(std::filesystem::exists(target.resolved));
    REQUIRE(files_in(where.seeds()) == 0u);
}

TEST_CASE("The seed is unchanged whatever is read and written", "[examples][documents]")
{
    const scratch where("the-seed-is-never-written");
    author(where.seeds() / "machine.xml", authored);

    const demo::documents mine(where.seeds(), where.state());
    for(int again = 0; again < 4; ++again)
    {
        static_cast<void>(mine.reading("machine.xml"));
        static_cast<void>(mine.writing("machine.xml"));
        static_cast<void>(mine.reading("machine.xml"));
        static_cast<void>(mine.writing("supplied-chain.xml"));
    }

    REQUIRE(bytes_of(where.seeds() / "machine.xml") == std::string(authored));
    REQUIRE(files_in(where.seeds()) == 1u);
}
