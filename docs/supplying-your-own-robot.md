# Supplying your own robot

praxis publishes the mechanism that turns a robot description into a preset. It publishes no robot.
This page is for a project that is not this repository's demonstration and that ships a robot of its
own: what you declare, what you deploy and where, how a description path written in your document is
resolved against the package roots you supply, and what happens when none of them holds it.

Everything below is stated against the sources named beside it, so each claim can be checked where it
is made.

## praxis carries no description

No target in this repository declares a robot-description resource of its own, and no description or
mesh file is tracked here. The two descriptions the demonstration loads are declared in
`cmake/dependencies.cmake`, inside an `if (PRAXIS_BUILD_EXAMPLES)` block. That option is declared in
the top-level `CMakeLists.txt` and defaults to `PROJECT_IS_TOP_LEVEL`, so it is off in a project that
adds praxis with `add_subdirectory` or FetchContent, and such a project downloads neither.

`meios_declare_resource` resolves and downloads its tree at configure time, which is why the guard
sits around the *declaration* rather than around the deployment: a declaration that is reached costs
a fetch whether or not anything deploys what it fetched.

What you supply instead is three things: a resource declaration, a deployment attached to your own
executable, and a list of package roots handed to `praxis::presets::register_arms`. The rest of this
page is those three, in that order.

Both CMake functions come from meios. praxis brings meios in with `FetchContent_MakeAvailable` in
`cmake/dependencies.cmake`, and meios defines the two functions in its own top-level listfile and in
its installed package configuration, so they are in scope in your project once praxis has been added
either way.

## Declaring the resource

`meios_declare_resource` acquires a named directory tree of data — parsed, never compiled, never
linked — and registers it under `NAME` for later deployment. Give it `NAME` and exactly one
acquisition mode:

| Argument | Meaning |
| -------- | ------- |
| `NAME` | a plain identifier the deployment refers to the tree by |
| `GITHUB` with `REF` | an owner/repository pair and a tag, branch or commit, served as an archive |
| `URL` with `HASH` | an archive by address, pinned by content hash |
| `GIT_REPOSITORY` with `GIT_TAG` | a clone, which is what LFS, submodules and private auth need |
| `SOURCE_DIR` | a tree already on disk; nothing is fetched |
| `SPARSE_PATHS` | the subtrees to check out, rather than the whole tree |
| `SUBDIR` | descend into this directory of the acquired tree and treat it as the tree |
| `OUT_DIR` | a variable to receive the resolved tree path |
| `STRIP_TOP_LEVEL` | drop the single wrapper directory an archive unpacks into |

Exactly one of `SOURCE_DIR`, `URL`, `GIT_REPOSITORY` and `GITHUB` may be given. `GITHUB` is sugar
over the archive path — the wrapper directory GitHub puts around the tree is stripped without being
asked for — so a `GITHUB` declaration and a `URL` declaration behave alike from there on.

Slicing a tree needs the git protocol, because no archive endpoint serves part of one. `SPARSE_PATHS`
is therefore rejected beside `URL`, `SOURCE_DIR` or `HASH`, and a `GITHUB` declaration that carries it
becomes a clone of the same repository with `REF` as the branch or tag. The clone is shallow and
selects the revision with `git clone --branch`, so pin it with a tag or a branch rather than with a
bare commit, and note that cone-mode sparse checkout needs git 2.28 or newer. A pattern that selects
nothing is reported rather than passed over.

An archive pinned by `HASH` is reused from the resource cache; an unpinned archive names where bytes
came from but nothing about what was served, so a tree fetched under one is re-fetched on every
configure rather than reused. The cache sits under the build directory unless
`MEIOS_RESOURCE_CACHE_DIR` points it somewhere several build trees can share.

```cmake
meios_declare_resource(
    NAME my_arm_description
    GITHUB my-lab/my_arm_description
    REF 8c2f1d0b7a6e5c4d3b2a190807060504030201ff
    HASH SHA256=<the archive's hash>
)
```

Acquisition can be redirected per resource without editing the declaration: configuring with
`-DMEIOS_RESOURCE_my_arm_description_SOURCE_DIR=<path>` takes the tree from disk instead, which is
what an offline or air-gapped build uses.

## Deploying it beside your binary

`meios_target_deploy_resources` places declared trees where a built target expects them. It takes the
target, `RESOURCES` (declared names), `PACKAGES` (which subtrees of those names to place) and
`SUBDIR`, which is relative to the target's runtime output directory.

```cmake
meios_target_deploy_resources(my_workbench
    RESOURCES my_arm_description
    PACKAGES urdf meshes
    SUBDIR packages/my_arm_description
)
```

Two rules decide the layout, and both are worth knowing before you pick a `SUBDIR`.

Each named entry keeps its own directory name at the destination. A `package://<name>/…` reference
resolves to `<package root>/<name>/…`, so copying a package's *contents* into the package root would
strip the very name the reference is looked up under. Which end of the pair carries the package name
therefore depends on the shape of the tree you fetched:

- The tree **is** one package's contents — it holds `urdf/`, `meshes/` and the like at its top. Name
  those subtrees in `PACKAGES` and put the package name at the end of `SUBDIR`, as above. The files
  land at `<runtime directory>/packages/my_arm_description/urdf/…`, and `packages` is the package
  root.
- The tree **holds** one directory per package. Name the packages in `PACKAGES` and let `SUBDIR` be
  the package root alone — `SUBDIR packages` — because each entry already brings its own name.

Several resources may share one `SUBDIR`, and that is how sibling description packages end up under a
single package root: declare each one, deploy each one with the same `SUBDIR`, and they become
siblings there.

Attaching the deployment to the target rather than to a directory is what puts the files beside the
binary, which is where the process resolves them from at run time.

## Resolving a path your document names

Your document names a description under the `description/path` key. Your program hands
`praxis::presets::register_arms` a span of package roots. praxis walks that span in the order you
supplied it and takes the first root under which the joined path exists; that is the whole rule
(`presets/arm/arm_document.cpp`).

- The test is `std::filesystem::exists` on `root / path`. praxis applies no case folding, no Unicode
  normalization and no containment test of its own, so the comparison is the filesystem's — a
  spelling that differs only in case resolves on a case-insensitive filesystem and does not resolve
  on a case-sensitive one.
- There is no fallback to the directory the document itself was read from. A root is the only thing a
  relative description path is joined to.
- An absolute description path replaces the root instead of joining to it, so it is tested as itself
  and the roots decide nothing.
- A path that no supplied root holds is carried on as the document wrote it, so the failure names the
  spelling somebody typed rather than a place that was never chosen.
- A caller supplying an empty roots span gets that last outcome for every document, since there is no
  root to walk.

A path no root holds still registers its preset. Registration asks only that the document loads, that
it states a name no document read before it has taken, and that it names a description at all
(`presets/arm/arm_registration.cpp`); whether that description loads is settled later, when the preset
is composed. The composition then names what it could not load:

```
Loading description <path> failed: <message> with code <n>
```

That is the message `presets/arm/arm.cpp` emits, and the composition yields no preset content after
it. The split is deliberate and worth relying on: a preset that vanishes leaves nothing to act on,
while a preset that appears and names its own failure is something a person can see and fix. A
document that names no description at all is a different case — it offers nothing, and registration
says so:

```
The document <path> offers no preset: it names no description
```

References *inside* the description are a separate job with the same input. praxis copies the roots
you supplied into `meios::load_options::package_roots`, and meios resolves every `package://` and
`$(find …)` reference in the description and its includes against that same list. One list serves both
the description path and everything the description points at.

## A worked example, end to end

**The listfile.** Add praxis, declare your resource, build your executable and deploy the tree beside
it.

```cmake
include(FetchContent)

FetchContent_Declare(
    praxis
    GIT_REPOSITORY https://github.com/skrede/praxis.git
    GIT_TAG v0.1.0
)
FetchContent_MakeAvailable(praxis)

meios_declare_resource(
    NAME my_arm_description
    GITHUB my-lab/my_arm_description
    REF 8c2f1d0b7a6e5c4d3b2a190807060504030201ff
    HASH SHA256=<the archive's hash>
)

add_executable(my_workbench main.cpp)
target_link_libraries(my_workbench PRIVATE
    praxis::praxis praxis::manipulator_presets praxis::manipulator
    praxis::trajectory praxis::rigid_motion praxis::config)

meios_target_deploy_resources(my_workbench
    RESOURCES my_arm_description
    PACKAGES urdf meshes
    SUBDIR packages/my_arm_description
)
```

**The document.** Its root element is `<arm>`, and every key path in it is one
`praxis::presets::arm_keyspace()` declares, so you never spell a key path of praxis's own. The
description path is relative to a package root.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<arm>
    <preset name="my arm" scenario="every window"/>

    <description path="my_arm_description/urdf/my_arm.xacro" evaluation="fail" missing_asset="fail"/>

    <initial>
        <joint index="0" degrees="0"/>
        <joint index="1" degrees="0"/>
    </initial>
</arm>
```

A description that takes xacro arguments carries them as children of the same element. A name set
here wins over the document's own `<xacro:arg>` default:

```xml
<description path="my_arm_description/urdf/my_arm.xacro" evaluation="fail" missing_asset="fail">
    <argument index="0" name="variant" value="long_reach"/>
</description>
```

`scenario` takes one of the spellings `praxis::presets::arm_scenario_labels()` returns; `evaluation`
and `missing_asset` each take `fail`, `warn` or `skip`.

**The program.** Form the package root relative to your own binary's directory — the deployment put
the tree there — collect the documents you offer, and register.

```cpp
#include "praxis/presets/arm_registration.h"

#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

int main(int, char **argv)
{
    const std::filesystem::path beside = std::filesystem::weakly_canonical(std::filesystem::path(argv[0])).parent_path();

    const auto registry = std::make_shared<praxis::scene::preset_registry>();

    const std::array<std::filesystem::path, 1> roots{beside / "packages"};
    const std::vector<praxis::config::location> documents{praxis::config::resolve("my_arm.xml", beside)};

    const std::vector<std::string> registered = praxis::presets::register_arms(registry, documents, roots, {}, {});
}
```

The fourth and fifth arguments are the two routes declared in `praxis/presets/routes.h`. A
`document_route` is asked where a named document is read from at the moment a composition wants it,
which is what a caller whose documents can move between a shipped copy and a written-back one
supplies; a caller whose documents have one place each passes nothing, and every composition then
reads the location its preset was registered with. A `composed_route` is told what a composition was
built from once it has been answered, which is where an edit is written back; a caller with nowhere to
write passes nothing. Both are `std::function`, and an empty one means "no route", not "an error".

The five-argument form binds the library's own reference implementations. The eight-argument form
takes the three capability sets explicitly, which is how you bind your own:

```cpp
const praxis::manipulator::capabilities arm     = my_arm_capabilities();
const praxis::trajectory::capabilities shapes   = praxis::trajectory::baseline();
const praxis::rigid_motion::capabilities motion = praxis::rigid_motion::baseline();

const std::vector<std::string> registered =
        praxis::presets::register_arms(registry, documents, roots, {}, {}, arm, shapes, motion);
```

`register_arms` returns the names it registered, in the order it registered them, so a caller that
opens the first without naming one gets the first document it offered. A document that fails to load,
states no name, states a name an earlier document already took, names no description, or names a
scenario that is not offered contributes no entry and says why on the log.

## When it does not resolve

Three things to look at, in this order. Run the commands from the directory your executable was built
into, which is the directory the deployment wrote to and the one the process resolves against.

**What actually landed.** The deployment runs as part of your target's build, so check the runtime
directory rather than the fetched tree:

```shell
find packages -maxdepth 3
```

If the package root holds the package's *contents* rather than a directory named after the package,
the `SUBDIR` and `PACKAGES` pair is the thing to fix — see the two layout rules above.

**What path the document names.** Read it out of the document rather than from memory:

```shell
grep -n 'description path=' my_arm.xml
```

That path is joined to a root as written, so a stray directory component, a case difference from what
landed on disk, or a leading separator that makes the path absolute is enough to miss.

**What roots the process handed in.** They come from your own code, not from a file, so check the
value you build there and join it by hand against the path above:

```shell
ls packages/my_arm_description/urdf/my_arm.xacro
```

If that exists and the load still fails, the description resolved and something inside it did not. A
`package://` reference, or a `$(find …)` in an include, is resolved against the same root list, so a
package that was never deployed fails there and is named on the log with the reference that wanted
it.

For an acquisition that fails before any of this — a certificate error during configure, or an
archive whose bytes stopped matching its pinned hash — see the repository README's build notes on
pointing CMake's download path at a trusted bundle and its section
[If the KUKA description fails on a hash mismatch](../README.md#if-the-kuka-description-fails-on-a-hash-mismatch).
