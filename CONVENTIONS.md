# coding conventions

The goal is a codebase that is lean — small, readable, fast — with no loss of features, 
capability, or speed. These conventions apply to everyone who writes code here. This file
is authoritative for code style. When something here conflicts with habit, this wins.

## No planning artifacts

Planning lives in the planning tools, never in the code. Never reference an ID, key, or label
produced by a planning or issue-tracking tool — phase numbers, milestone names, plan or task
numbers, requirement IDs, invariant labels (`INV-2`, `SER-01`, …), or any planning-tool or issue-tracker
artifact — anywhere in the codebase: identifiers, comments, documentation, examples, and commit
messages.

State the *thing itself*, not its planning label. A comment explaining a real invariant says it
in plain words ("the core holds no platform-specific code"); it never cites "INV-2". If a reader
would have to open a planning document to decode a comment, the comment is wrong. The same goes
for narrative scaffolding from how the code was built — "the slice", "the gate", "this milestone"
— which means nothing to a reader of the finished code. Cut it.

## Language and portability

- Idiomatic C++20. Use the language; do not reinvent what the standard library, asio, or
  boost already do well.
- The code must compile and run on macOS, Linux, and Windows. Keep platform-specific code
  inside clearly isolated backends, never in the core.
- Follow the conventions of modern C++ libraries (standard library, asio, boost). Existing
  project conventions take precedence. If you are unsure, or a popular idiom contradicts the
  project, ask rather than guess.

## Macros

- Do not use macros to express the library's logic, behavior or API. praxis authors no
  function-like macros, no X-macro lists, and no token pasting or stringization to form
  identifiers or names.
- An identifier the preprocessor assembles appears in no source file, so nothing can index,
  navigate or complete it, and it is invisible to every tool that reads the code rather than
  the expansion. Say it in the language instead — enumerations written out, constexpr tables,
  templates, concepts.
- Macros an external header defines are that library's surface, not ours, and are fine to
  use: Catch2's `TEST_CASE` and `REQUIRE`, and anything else a dependency provides.
- Three uses of the preprocessor remain necessary here and stay permitted — include guards, as
  required below; conditional compilation for portability and feature-test guarding
  (`__cplusplus`, `__has_include`, `__cpp_lib_*`), which is what the include-leak gate is built
  from; and a fallback definition for a value the build system injects. Nothing else.

## The Foundation and extensions

The library is six **Foundation** targets — the language-floor backports, the configuration store,
the extension machinery every seam is described in, the scheduler that owns the clock, the preset,
rendering and GUI pipeline, and the harness an implementation bound to a slot is measured by — none
of which names any robotics, plus **extensions**, each owning everything specific to
one domain: its vocabulary, its mathematics, its backend logic, its stencil and its controls.
Beside those two kinds stands a third: `praxis::rigid_presets` and `praxis::manipulator_presets`,
targets that link extensions rather than being ones, composing their windows, stencils and inert
defaults into named scenarios and publishing from `presets/include` rather than from a module
include root. A consumer names the half it composes from, and the manipulator half does not link the
rigid one. They are linkable, exported and installed surface, and neither appears in a module list
in `lib/CMakeLists.txt`, so the two gates that read those lists — the Foundation vocabulary scans
and the link-isolation closure — do not reach them, and their header guards, file names and include
forms are held by review. The shipped surface record does reach them: the extraction globs
`presets/include` beside the module include roots, as an include root of its own rather than through
a module list. The declared-sources gate reaches them through a `presets/` section of its own, and
the include-leak assertion is applied to each target directly.

- No Foundation target may name a domain type.
- An extension is one target, carrying its vocabulary, its contracts, its inert defaults, its
  reference implementation and its scene contribution together. The reference is located by path —
  under a `baseline/` directory in the module's include root and its sources — rather than by target.
  The link-isolation gate, which runs as a test rather than at configure, reads the module lists out
  of `lib/CMakeLists.txt` and asserts for every module it finds there that no second target
  splitting off an extension's reference or its scenery is declared beside it, that the Foundation
  reaches no extension, and that an extension reaches none of those listed after it. What else a
  module may not reach is subject matter no rule derives, so each module names it in its own listfile as
  `PRAXIS_CLOSURE_ABSENT` and the gate stops on a module that names nothing. The dependencies it
  polices are written down nowhere: they are the union of what every module's link lines declare
  outside this repository's namespace, read from the listfiles, so one acquired today is policed on
  the next run. Which of them a module may publish through its own headers is subject matter again,
  so each names those as `PRAXIS_PUBLIC_DEPENDENCIES`, writes `NONE` where it publishes none, and a
  public link to a name that list omits stops the gate. The include-leak gate
  asserts at configure, per target, that the solver, the control library and the logging dependency
  are linked privately and reach no published header. The renderer and the GUI library are public
  surface on an extension that contributes to the scene, as is the model library on one that reads a
  robot description.
- An extension is authorable **outside this repository**: it declares its own capability, ships its
  own inert defaults and descriptor table against the exported `praxis::extension` machinery, and is
  reported by the coverage facility without any file here being edited.
- An extension links the Foundation; the reverse edge does not exist over any of its six targets,
  and the link-isolation gate asserts its absence in both directions.
- **Adding a domain means adding an extension, not editing a shipped one.** That is the whole return
  on the boundary and it is the sentence to check a change against. Inside this repository an
  extension is a module directory and one entry in `lib/CMakeLists.txt`, which names every module
  once and is the single authority both the build and the gates read.

Two consequences are concrete enough to write down. The generic window framework and the reusable
widgets are Foundation surface and carry no domain vocabulary at all. Every window that knows what it
is configuring belongs to the extension that owns the thing configured — a combo over a domain type is
a domain control however small it is.

`cmake/verify_conventions.cmake` enforces this over every Foundation target as two scans, each reading
the module lists out of `lib/CMakeLists.txt` so that what is scanned and what is built cannot drift.
The first is a word list: `robot`, `robotic`, `joint`, `tool`, `jog`, `flange`, `gripper`,
`manipulator`, `actuator`, `end_effector`, `kinematic`, `wrist`, `payload`, `waypoint`, `trajectory`,
`screw` and `tcp`. The second is the name of every extension the umbrella lists. Every header and
source of all four is scanned against both, and there are no exceptions.

Neither scan reaches an extension. Every extension either publishes a domain's mathematics or
implements it, so the word list would forbid its own subject matter and the extension names would
forbid it naming itself.

## File and function size

The limits, lines **including comments and blank lines** — the budget is the whole thing as
it sits on screen:

| Unit     | Target     | Hard ceiling |
| -------- | ---------- | ------------ |
| Function | 5–15 lines | 25 lines     |
| File     | ~100 lines | 200 lines    |

The limits govern `lib/`, `presets/` and `tests/`. **`examples/` is exempt from the file ceiling**:
a demonstration's shape is its table of what ships, and that table grows by one row per scenario
without any of its rows becoming a separate thing to read. Decomposing it to hold a number splits
one list in two. The function ceiling still applies there, and so does everything else in this
document.

A record a build step generates and rewrites whole is exempt from the file ceiling as well. Its
length measures the surface it describes rather than a shape anyone chose, and it has no
decomposition: the pieces a split made would be read by nobody and rewritten together on the next
extraction. The function ceiling does not reach one, which declares no functions.

Readability is the goal, not SOLID or DRY orthodoxy. Group code that is read together. Split
where it genuinely separates responsibilities — never to chase a number.

Each scope does **one thing**, and the smaller that one thing is, the cleaner the code reads: a
library does one thing, a namespace groups the types and functions working toward one thing, a class
does one thing, a function does one thing. A unit that runs over its ceiling is usually the "one
thing" drawn too broad — decompose it, do not widen the budget. (The genuinely-cohesive whole that
splitting would *harm* is the registered exception below, not the default.)

Forbidden:

- **Salami-slicing** — carving a cohesive unit into fragments just to fit a limit.
- **One-function files** — a file that exists only to hold a single function.
- **Artificial purity** — a wrapper or layer that adds indirection without separating a real
  responsibility.

Keep public headers lean by moving internal helpers into a `detail/` directory and a
`detail::` namespace. Make `detail/` files and `detail::` types as needed — but a `detail/`
file is still a real file and obeys these same rules.

Enforcement is review. No tool checks these numbers: `.clang-tidy` holds the project's lint
configuration but nothing runs it — no listfile invokes it and no target exists for it — and it
carries no file-size check either. The ceilings are a guideline a change is held to when it is read.
Asserting a gate that does not run is worse than having none, so if the check is wanted it arrives
together with the build wiring that runs it.

### Exceptions

A file or function may exceed its ceiling only when it is a single cohesive whole that
splitting would *harm* — scattering shared state across files, or forcing an artificial-purity
layer. An exception is a considered decision, not a fallback; prefer decomposition, and reach
for an exception only when decomposition would make the code worse.

Every exception is recorded in **`EXCEPTIONS.md`**, whose over-limit register is the complete list of
sanctioned over-limit units, each with its justification. That register is reviewed rather than read:
nothing reads it at build time. A row is an argument a reviewer accepted, an overage with no row is a
defect, and a row whose file has since dropped under the ceiling is stale and is removed. There is
**no in-code marker**: the justification is bookkeeping, and bookkeeping never belongs in the code.

## Comments

The codebase is written for people to read. A comment must say something the code itself
cannot. **The default is no comment** — clear names and signatures carry the meaning.

Write a comment only when it does one of:

- **(a)** cites a source — a paper, RFC, spec, URL, bug report, or standards clause;
- **(b)** names a non-obvious algorithm or technique so a reader can look it up;
- **(c)** explains a non-obvious design choice in one or two sentences — a bug workaround, a
  deliberate tradeoff, or a contract / ordering / side-effect / invariant that is real but not
  visible from the code.

Delete everything else. Never write:

- comments that restate a name, signature, type, or obvious control flow in more words;
- tables or lists that transcribe the lines of code beneath them;
- sign-posting or narration ("this section does X", "first we … then we …");
- bookkeeping or process markers — size-gate justifications, ledgers, "see the doc" pointers,
  anything whose audience is the maintainer's process rather than the code's reader. That lives in
  `EXCEPTIONS.md` or the planning tools, never in the source.

A comment that merely repeats the next line is noise: it steals screen space, competes with
the code, and rots out of sync. Cut it. Keep a multi-line comment tight — no blank `//`
separator lines padding it out.

A comment longer than a short tag goes on its **own line above** the code it explains, never
trailing. A long trailing comment crowds the statement and forces the formatter to wrap the code
beneath it into an ugly multi-line shape; the line above is where the eye looks first anyway. A
brief trailing tag on a data line — a unit, an enumerator's value — is fine where it reads as part
of that line.

Comments count toward the size budget. A file that is 30% comments is usually
30% over-commented, not a candidate for a bigger budget — trim first. Do not write a novel.

## Includes

Internal project includes (`#include "..."`) come first, third-party libraries second,
standard-library headers third. Order matters only when something must precede something else
for a real reason.

- The three major sections are separated by one blank line.
- Within a major section, group includes by folder, separated by one blank line. In the project
  section the file's **own subsystem comes first** (its own folder/module), then the remaining
  project folder groups. Only one blank line between any two groups.
- Within a folder group, sort by the include's **file-name length, shortest first**; file names
  of equal length sort alphabetically.
- A first-party header is included **with quotes wherever it is written** — in a module's own
  sources, in a sibling module, in a test, in the demonstration. The quoted form is what marks an
  include as this project's; angle brackets mark a dependency or the standard library. A test is not
  an exception: it is inside this repository, so it writes `"praxis/manipulator/types.h"`. The
  convention gate checks this.

## Naming

- **File names are `snake_case`.**
- **Type names are `snake_case` too** — `screw_chain`, `robot_controller`, `preset_registry`,
  `axis_order`. This matches the standard library and the sibling libraries the project is built on,
  rather than the mixed-case forms the tree carried before.
- A member holding a callback or hook (a `move_only_function`) takes a `_cb` suffix and groups with
  the other callbacks in the member list — the suffix marks it as an injected behavior, not state.
- That rule names a C++23 facility the language floor puts out of reach, so it does **not** apply to
  the plain function pointers the extension seam is built from. An injected operation is named for
  the operation it performs — `skew_symmetric`, `task_space_pose` — and carries no decoration; the
  aggregate it sits in is what marks it as injected.

## Construction

- Initialize a class's members in the **constructor's init list, never in-declaration** — one place,
  one source of truth, so two defaults can never drift apart. A class with a constructor carries no
  in-declaration member initializers.
- A member wanting a default value is exactly what a constructor is for: **give the type a constructor**
  and put the default in its init list. Do not leave the default in-declaration. The only type without a
  constructor is a pure aggregate that carries *no* defaults at all — brace-initialized at every use, no
  in-declaration initializers; it has nothing to move. The moment one member wants a default, the type
  takes a constructor.
- **One carve-out, and it covers exactly one thing: an aggregate whose purpose is
  designated-initializer composition keeps its in-declaration member initializers.** That is the
  injection aggregates — the per-capability `*_ops` a seam is built from, and the per-extension
  `capabilities` that composes an extension's own — and nothing else.
  Giving them a constructor would eliminate the property that made them worth choosing, because a
  type with a user-declared constructor is not an aggregate and cannot be written with designated
  initializers at all, while a type with default member initializers still is one. The defaults are
  the point rather than a convenience: every slot defaults to a named inert function, so a
  value-initialized aggregate is a table of working inert implementations instead of a table of
  null pointers waiting to be called through.
- **The constructor's init list is written in member-declaration order.** Members initialize in
  declaration order regardless of the init-list order, so the two must agree (no `-Wreorder`). This
  has a consequence for the member triangle below: a member consumed by another member's constructor
  must be **declared before** its consumer, and that initialization dependency **overrides the length
  triangle** — the triangle is the shape where dependencies leave it free, never at the cost of a
  correct construction order.

## Class layout

Order a class body so the data is visible before the operations on it:

```
class T
{
    // leading private nested helper types (if any)

public:
    constructor(s)
    public functions

private:
    members          // data first, right after private:
    private functions
};
```

A private nested helper type a class needs may lead the class body, before `public:`. The member
triangle (above) orders the members within the leading private data block, subject to the
initialization-order constraint in **Construction**.

### Polymorphic bases

An interface the library ships writes its five special members out: defaulted default constructor,
deleted copy construction and copy assignment, deleted move construction and move assignment, virtual
defaulted destructor. Stopping at the destructor — which is what every polymorphic base in this tree
used to do — silently suppresses the move operations and leaves copy implicitly declared on a type
that must never be sliced. The compiler's non-virtual-destructor and overloaded-virtual warnings are
now on, so the form is decided here rather than left to each author.

## Types and attributes

- **Spell out return types.** Do not write `auto` as a function's return type — the named type is
  what a reader looks for first, and `auto` makes the code harder to read and understand. The only
  exception is a return type that is genuinely unspellable (a lambda / closure), where `auto` is
  unavoidable. `auto` for a *local variable* is fine where the initializer makes the type obvious.
- **No `[[nodiscard]]`. Ever — with one exception, named here and nowhere else.** Do not decorate
  declarations with attributes the compiler does not require — they are visual noise. Keep attributes
  to the few the language genuinely needs. The exception is the owned `expected` class template and,
  separately, its `void` partial specialization, which does not inherit the primary template's
  attributes: both carry the attribute at **class** level, because a dropped result is a reported
  failure silently turned into a no-op, and one annotation on the type covers every fallible return
  where a per-declaration sweep would drift out of sync with the surfaces it guards. It is written on
  those two types and on nothing else — not on an accessor, a size or empty query, or a state
  predicate, and never at a call site or on an individual declaration.

## Header guards and namespaces

- Use header guards, not `#pragma once`. The form is `HPP_GUARD_PRAXIS_<FOLDER>_<FILE>_H`, derived
  from the header's own path with the namespace directory dropped once:
  `praxis/manipulator/types.h` gives `HPP_GUARD_PRAXIS_MANIPULATOR_TYPES_H`, and
  `praxis/trajectory/pose_trajectory.h` gives `HPP_GUARD_PRAXIS_TRAJECTORY_POSE_TRAJECTORY_H`. A
  header under a module's `src/` derives the same way, with the module's name where the include path
  would have put it: `lib/praxis-manipulator/src/robot/mesh_library.h` gives
  `HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_MESH_LIBRARY_H`. The path carries a module segment already, so
  the rule needs no special case for one.
- Namespaces are **flat for the vocabulary every extension shares and nested for everything an
  extension or a module owns**. Flat `praxis::` carries two things and nothing else: the spatial
  value types every extension speaks — transforms, rotations, twists, screw axes, adjoints — and the
  machinery a capability is described in, the slot descriptor, the capability view, the slot set and
  the coverage functions. That machinery's module is `praxis::extension` as a target and an include
  root and declares no namespace of that name, because what it publishes is the vocabulary every
  extension is written in rather than something an extension reaches into. An extension's own
  vocabulary, its contracts, its inert defaults and its reference sit under that extension's
  namespace: `praxis::rigid_motion` with `praxis::rigid_motion::inert`, `praxis::trajectory` with
  `praxis::trajectory::inert`, `praxis::manipulator` with `praxis::manipulator::inert`. Everything
  else is nested under the module that owns it: `praxis::scene`, `praxis::presets`, `praxis::demo`,
  and `praxis::detail` for what is not surface at all. In a source-distributed library the namespace
  level is the only public-versus-internal boundary there is, so anything left unqualified is a
  promise.
- Do not write a `// namespace X` comment after a closing namespace brace.
- Do not write a comment after `#endif` restating the guard macro.

## API design

- No raw pointers in public APIs. Pass by `const&` (or by value, `std::span`, or a smart
  pointer where that is the right ownership story).
- Distinguish three things and never conflate them: **required** (no default),
  **required-with-default** (the override is optional), and **`std::optional`** (absence is
  itself meaningful). Do not use `std::optional` as a stand-in for a default.
- Pre-release, there is no `[[deprecated]]`. Delete a superseded type outright — there are no
  external users to cushion.

## Lifetimes and ownership

- Structural, single-owner lifetimes. The owner sequences teardown.
- **Raw pointers are an anti-pattern — avoid them as far as possible.** Use a `unique_ptr` for sole
  ownership and a `shared_ptr` for genuinely shared ownership; for everything else, pass and store
  **references**. The system composes this way: an owner holds the long-lived utilities and hands
  them out by reference (a context object the parts reach through), so a non-owner never needs a
  pointer. A raw pointer survives only where absence is a real, modelled state (a nullable lookup
  result) — and even there prefer a reference or `std::optional` where it fits.
- A non-owning relationship that *must outlive* its holder is a **reference, not a raw pointer** — a
  reference says "not optional, not owned" without the nullability a pointer implies. Store it as
  `std::reference_wrapper<T>` when it lives in a container that needs assignability (a `std::vector`
  element, say). Reach for a smart pointer only when there is real shared or transferred ownership.
- **A function pointer stored as an injected operation is not the raw pointer this section
  forbids.** The rule above is about ownership of objects, and a function pointer owns nothing. The
  slots in the extension aggregates are values, never null, and they are the seam's whole mechanism;
  do not flag them.
- No `std::shared_from_this` / `std::enable_shared_from_this`.
- No per-callback liveness guards that paper over a posted task outliving its target. If a
  posted closure can run after its target dies, fix the ownership and teardown sequencing —
  do not add a flag to guard every access.

## Formatting and tooling

- `.clang-format` is the formatter, and it is authoritative on its own for the mechanical rules:
  185-column lines, comments never reflowed to fit the column, short functions never auto-collapsed
  onto one line (the author decides), and includes left unsorted so the hand-ordering below holds.
  Run it.
- `.clang-tidy` is the linter's configuration. It is **not** wired into the build and it carries no
  size check; run it by hand and read what it says.
- What does run, on every configure, is `cmake/verify_conventions.cmake`: the header-guard form, the
  file naming, the include mechanism, the closing-brace comments, the repository's root shape, the
  domain-vocabulary rule, the synchronization allowlist, the angle unit every shipped header's
  declarations spell in their own names — screw pitch excepted, being an axis translation per radian
  rather than an angle, and the two rotation-triple slots named for the Euler angles excepted by a
  written list, each being a name the coverage facility reports and an enumerator spells rather than
  a spelling free to move — and the slot tables' freedom from a scheduling type. The angle scan reads a
  line rather than a declaration, so a declaration wrapped across two lines, and a line pairing an
  unspelled angle with a spelled one, are outside what it sees. That exclusion list is read back: an
  entry on it that silences no line the scan would otherwise report stops the configure. It is a
  script, so a hook or a pipeline runs it unchanged:
  `cmake -DPRAXIS_SOURCE_DIR=. -P cmake/verify_conventions.cmake`.

  The synchronization scan carries the one register that *is* read. A lock is never the discipline by
  which two strands share an object, but a hand-off across a boundary is built from one, so the
  primitives are named in a per-file allowlist written beside the module lists in `lib/CMakeLists.txt`
  and argued for in `EXCEPTIONS.md`'s synchronization register. A primitive appearing outside that
  list, and an entry whose file is gone or no longer carries one, each stop the configure.
- `cmake/verify_declared_sources.cmake` runs there too, and it answers the question no build step
  asks: it diffs each module's source list against the files under that module in both directions, so
  a header that exists but is compiled by nothing, and an entry naming a file that is not there, stop
  the configure by name rather than surviving to a link or to nothing at all.
- `cmake/verify_snapshot_freshness.cmake` runs there too. A snapshot row naming a header that is not
  in the tree describes a tree that does not exist, so it stops the configure and names the header.
- The other two run as tests, because both need a configured tree and one of them regenerates the
  link graph, which no configure can do inside itself. `ctest` gives
  `cmake/verify_link_isolation.cmake` a scratch build directory of its own — run by hand it
  reconfigures whatever build directory it is given, so give it one you do not mind — and runs
  `cmake/verify_public_surface.cmake` against the shipped headers directly.
- `cmake/verify_public_surface.cmake` needs Universal Ctags and fails rather than skipping without
  it. Its test is registered either way, because a test that is not registered is silent in the
  report and a report that says nothing reads as a report that found nothing. The snapshot in
  `tests/golden/` is only ever written from that extraction, because a hand-assembled snapshot is
  indistinguishable from an extracted one. Regenerate it whenever a change moves or renames a
  shipped declaration.
- `PRAXIS_ENABLE_CTAGS_GATES` governs the surface gate alone, which is the only one needing the tool.
  It defaults on for this tree and off for a project that adds praxis. Off, every other gate and every
  unit test still runs, so a machine without the tool can run the suite without checking the surface.
  Continuous integration sets it on explicitly, and a developer whose machine lacks the tool sees the
  gate fail by name rather than a report that omits it. The three configure-time gates read this tree
  rather than a consumer's and run only where it is the top-level project.
- Apply the formatter across the whole tree. A change is not done until it is clean.

### Shape (readability guidelines, not rigid rules)

These shape the code for the human eye. They are judgment calls — a case that reads better the other
way is a fine exception. The intent is the rule; the shapes are how it usually looks. The point is to
make the next name easy to find.

- **Downward waterfall.** When a signature or declaration wraps, each continuation line is *longer
  than or equal to* the one above it, so the wrapped arguments slope downward and the eye falls to
  the next one.
- **Declaration order in a shipped injection aggregate is frozen**, and this is a rule rather than a
  shape. A designated initializer must name members in declaration order, so reordering a slot breaks
  every downstream project written against the previous tag, while appending or inserting one does
  not. The member triangle below pushes toward exactly that reorder whenever a shorter declaration is
  added, and here it yields: a new slot appends within its capability group and the existing order
  does not move.
- **Member triangle.** Order a class's members by overall declaration-line length, shortest first —
  the type name dictates the order — so the members form a downward slope. A **single space** precedes
  each name; names are *not* aligned into a column (only consecutive `=` assignments are aligned).
  Semantic grouping wins when it helps: keep the callbacks together even if a shorter line lands among
  them. The triangle yields to initialization order (see **Construction**); shape serves reading,
  never the reverse.
- **Logical blank lines.** A blank line inside a function separates logical parts — set-up from the
  work, one phase from the next — the way a paragraph break does. Separate each member function from
  the next with a single blank line; the leading data members are packed together with none.

## English

American spelling: "stabilizing", not "stabilising"; "color", not "colour".

## Commit messages

Format:

```
{Prefix}: {summary sentence}.

- {what was done, one line per item}
- {another item if applicable}
```

The summary line is brief and descriptive; the bullet list expands on it. A single-item commit
may omit the bullets. Allowed prefixes: **Feature, Fix, Refactor, Docs, Examples, Optimization,
WIP**. Use `WIP` when the commit does not compile.

That is the whole list, and this is the only place it is written down, because three copies of a
list is how a list acquires a fourth entry nobody agreed to. A commit that would have been a build
or a test commit is a `Refactor` or a `Feature` by what it does to the project.

The no-planning-artifacts rule applies to commit messages too: never reference a phase number,
plan or task ID, requirement or invariant label, or any other planning-tool artifact in a commit
message.
