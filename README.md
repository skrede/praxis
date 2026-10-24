# praxis

A C++20 library for teaching the mathematics of robotics: rigid motions, serial manipulators,
trajectory generation and the iterative solves beneath them. Every capability a student is meant to
implement is a seam: an injected operation the library calls through, with an inert default
already in place, so the library builds and runs whether or not the student has written anything yet.
Filling a seam is writing one function and naming it in a designated initializer — no subclassing, no
rebuild of the library, and no graphics context needed to test it.

Three demonstration applications ship beside the library, `rigid_demo`, `manipulator_demo` and
`optimization_demo`. Each offers the scenarios one preset target composes, renders them, and drives
all of it through the same seams a student fills, so an implementation can be seen as well as tested;
only `manipulator_demo` loads a robot description and renders an arm.

## Modules

The library is six **core** modules — the compatibility floor, the configuration store, the extension
machinery every seam is described in, the scheduler, the preset, rendering and GUI pipeline, and the
evaluation harness — none of which names any robotics, plus four **extensions**.
An extension is a single
target that carries its own vocabulary, its contracts, the inert defaults behind them, its reference
implementation and its scene contribution together; the reference sits under a `baseline/` path
rather than in a target of its own. A logging dependency it needs is linked privately and reaches
none of its published headers, while the renderer and the GUI library
are public surface on an extension that contributes to the scene, as is the model library on one
that reads a robot description. An extension
can be authored outside this repository: adding a domain means adding an extension, not editing a
shipped one. Inside this repository it is a module directory beside the ones below and one entry on
one line of `lib/CMakeLists.txt`, which names every module once and is the single authority the
build and the gates both read.

| Module | Kind | What it contributes |
| ------ | ---- | ------------------- |
| `praxis::compat` | core | the stand-ins the C++20 floor needs for library facilities a later standard supplies: an expected-shaped return and the callable trait behind it, each behind its feature-test macro so a toolchain carrying the standard facility uses that one instead. It links nothing at all |
| `praxis::config` | core | the configuration store: the vocabulary a setting is declared in, the document behind it, the binding from a key path to a value, and the reader and the writer over both. The parser and the logging dependency are linked privately and reach none of its published headers |
| `praxis::extension` | core | the vocabulary a seam is described in: slot descriptors, capability views, slot sets, and the coverage report saying which slots still hold their inert default. It links nothing at all, and it is exported so a project outside this repository can declare a capability of its own |
| `praxis::scheduler` | core | the strand model everything else runs on: clocks, tasks, strands, the snapshot a strand publishes, the ownership gate that is the only route to a value a strand owns, and the rejection and overrun reports a refused or late task lands in |
| `praxis::scene` | core | the renderer, the window framework, the preset registry and the reusable widgets — general enough that a second domain reuses them unchanged |
| `praxis::evaluation` | core | what an implementation bound to a slot is measured with: the project tolerance, the residual kinds a discrepancy is reported in, the comparators over them, the seeded case source that draws a slot's inputs from the bulk of its range or from the neighborhood where its mathematics degenerates, and the per-slot report they assemble into. It names no domain — what it measures is whatever a capability published |
| `praxis::optimization` | extension | the iterative-solve seam whole: its vocabulary — a problem in residual form, given by its error, its Jacobian and its stopping test, or in objective form, given by its objective, its gradient, an optional Hessian and a stopping test over the gradient, either with optional bounds on its coordinates — the solver slot and the Newton and gradient-descent minimizer slots with their descriptors and an inert default behind each, and the reference implementations behind them: Newton-Raphson written from Lynch & Park, Modern Robotics, sec. 6.2.1, Newton's method and fixed-step gradient descent written from Nocedal & Wright, Numerical Optimization, sec. 2.2, and a projected Levenberg-Marquardt solve from Kanzow, Yamashita & Fukushima (2004) that keeps every iterate inside the bounds; and the scene contribution: the curve and surface scenarios, which solve a function of one or two variables for a root by the bound solver or minimize it by a bound minimizer on a strand of their own, the function either one a consumer registers by name with its derivatives written out or one typed in, with a parameter per letter it uses besides its variables and its derivatives taken exactly. A curve is drawn in a plot and a surface in the scene as a height field with every iterate on it, and their windows are these. Function: the source, Registered or Manual, with the registered function selected or the typed field, its reading under it and a slider and a range per parameter, and the range drawn over. Graph, for a curve: the plot, with every iterate, the selected step's construction and the ends of the bounds, where a Shift-click runs from the abscissa pressed. Display: switches for the iterates, the selected step and the bounds, and over a surface for the field and its zero curve or, minimized, its level curves and gradient. Solve: the seed, the iteration limit and, in the minimum form, the method and the step length; Run, Step once and Reset; a slider selecting the step; and the table of iterates. Linearization: a point by field and slider, the value and the derivative or gradient there, Show and Reset, and where the composition opts in, a registered derivative checked against a central difference. It names no robotics and links no other extension |
| `praxis::rigid_motion` | extension | the rigid-motion seam whole: the pose vocabulary every extension speaks — transforms, screw axes, angle conversions, Euler axis orders, the project tolerance — the frame and screw slots with their descriptors and an inert default behind each, and the reference implementations behind them, written from Lynch & Park, Modern Robotics, ch. 3; and the scene contribution: a stencil rendering a set of objects, each with its coordinate frame; the frame window whose Euler controls drive those frames, the roster creating, naming and removing them, and the selector choosing which of them the windows beside it are about; a readout of a frame's placement as a matrix, relative to its parent or to the world; and five windows, each driving the geometry a scenario draws from it: the screw window, an axis with its pitch and the angle turned about it; the twist-axis window, a twist and the screw axis it names; the two-pose window, two poses and the screw carrying one to the other; the rotation-axis window, an axis and an angle about it; and the decoupled-path window, the path carrying orientation and position between two poses apart from each other; the screw, twist and axis controls those windows are built from; and the declaration, reading and writing of an arrangement of frames in a document. The scenarios composing them are in `praxis::rigid_presets` |
| `praxis::trajectory` | extension | the trajectory seam whole: its own N-DOF vocabulary — a configuration vector and the bounds on it — the time-scaling, path, SE(3) pose-trajectory and via-point slots with their descriptors and an inert default behind each, and the reference implementations behind them, written from Lynch & Park, Modern Robotics, ch. 9, from Biagiotti & Melchiorri, Trajectory Planning for Automatic Machines and Robots, eq. (2.3), and built on the rigid-motion reference. It makes no scene contribution of its own: its slots are handed to the arm scenarios composed in `praxis::manipulator_presets` |
| `praxis::manipulator` | extension | everything specific to serial arms: its own joint vocabulary — joint vectors, Jacobians, the screw chain and the joint bounds, the last two of them names it gives the trajectory extension's types — the forward-kinematics, differential-kinematics, inverse-kinematics, robot, motion, modeling and task-space via-point slots with their descriptors and an inert default behind each, and the reference implementations behind them, written from Lynch & Park, Modern Robotics, ch. 4-6, alongside this extension's own screw-chain derivation, and built on the rigid-motion reference; and the scene contribution: that derivation from a robot description, the scene adapter, the controller and playback, and the operator windows |

`praxis::praxis` is a convenience target over the Foundation, and `praxis::extensions` one over every
extension shipped here. The named scenarios the demonstrations offer are composed in three preset
targets beside the extensions, each publishing from `presets/include`: `praxis::rigid_presets`,
`praxis::manipulator_presets` and `praxis::optimization_presets`, the last composing the robot-free
scenario over the functions a consumer registers with it. A consumer wanting a scenario links the
preset target that composes it, and no preset target links another.

## Build

A tree built as the commands below build it — tests and demonstration included — is large: about
19 GB for a Debug tree and about 5 GB for a Release one. Running the suite adds a few hundred
megabytes on top of that, because the link-isolation gate configures and builds a tree of its own.

The first build is long: tens of minutes rather than minutes. A Release tree configures, builds and
runs its whole suite in about a quarter of an hour at four parallel jobs on a recent multi-core
desktop, and fewer cores take proportionally longer. Almost all of it is compiling every dependency
from source alongside the library; builds after the first are incremental and far shorter.

Most of what is fetched is cloned by git and goes through git's certificate store, but one robot
description is downloaded as an archive through CMake's own download path, which verifies against a
trust store of its own. The two can disagree — behind a TLS-intercepting proxy, or on a CMake that
ships with no trust store at all — so a machine where `git clone` succeeds can still fail during
configure with a certificate error. Point the download path at the bundle git already trusts by
configuring with `-DMEIOS_RESOURCE_TLS_CAINFO=<path to the bundle>`.

```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

Five options gate what is built and what is checked, and one cache value selects the language
standard. Four of the options default to on when praxis is the top-level project, so a project that
adds praxis gets none of them:

| Option | Default | Effect |
| ------ | ------- | ------ |
| `PRAXIS_BUILD_TESTS` | top-level | the unit tests, run with `ctest --test-dir build` |
| `PRAXIS_BUILD_EXAMPLES` | top-level | the `rigid_demo`, `manipulator_demo` and `optimization_demo` executables |
| `PRAXIS_BUILD_DISPLAY_TESTS` | top-level | the tests that need a display, run under `xvfb-run` where it is installed and on the machine's own display where it is not |
| `PRAXIS_BUILD_CONSUMER_TESTS` | off | the rebuild-avoidance gate, which configures and builds a consuming project of its own |
| `PRAXIS_ENABLE_CTAGS_GATES` | top-level | the public-surface gate, which needs Universal Ctags |
| `PRAXIS_CXX_STANDARD` | `20` | the language standard this tree compiles at; 20 is the floor, the default, and the only value this tree is tested at |

`rigid_demo` offers the frame workbench, the two Euler-angle scenarios, and the screw, twist-axis,
two-pose and rotation-axis scenarios. It carries no robot description and no mesh file, and needs
none.
`optimization_demo` offers the robot-free scenario's presets as `praxis-optimization.xml` lists
them: root finding on a cubic and on a paraboloid, minimization of x^4 - 3x^2 + x and of
(x^2 + 10y^2)/2, and a typed function of one variable and one of two, each with a parameter. It
registers four curves and three surfaces, each with its derivative or gradient written out beside it
and one of each with its second derivative or Hessian as well, binds the reference implementations,
and has the Linearization window check the registered derivatives. Each preset reads the document
it ships with until it is saved, and the saved copy from then on. It needs no resource either.
`manipulator_demo` offers the arm presets and carries their resources: the robot descriptions and
mesh files are fetched during configure and deployed next to that executable as part of its build.
Each entry point resolves what it reads relative to its own binary, so run each from its own output
directory:

```shell
cmake --build build --target manipulator_demo -j
cd build/examples/manipulator && ./manipulator_demo
```

```shell
cmake --build build --target rigid_demo -j
cd build/examples/rigid && ./rigid_demo
```

```shell
cmake --build build --target optimization_demo -j
cd build/examples/optimization && ./optimization_demo
```

A project consuming praxis with `add_subdirectory` gets neither the tests nor the demonstration,
because both options default off unless praxis is top-level.

## Checks

Six scripts run the gates the architecture rests on. Each takes the source directory, and the three
that need a build tree of their own take that as well:

```shell
cmake -DPRAXIS_SOURCE_DIR=. -P cmake/verify_conventions.cmake
cmake -DPRAXIS_SOURCE_DIR=. -P cmake/verify_declared_sources.cmake
cmake -DPRAXIS_SOURCE_DIR=. -P cmake/verify_snapshot_freshness.cmake
cmake -DPRAXIS_SOURCE_DIR=. -DPRAXIS_BUILD_DIR=build/gate-probe -P cmake/verify_link_isolation.cmake
cmake -DPRAXIS_SOURCE_DIR=. -DPRAXIS_BUILD_DIR=build/gate-probe -P cmake/verify_public_surface.cmake
cmake -DPRAXIS_SOURCE_DIR=. -DPRAXIS_BUILD_DIR=build/consumer-probe -P cmake/verify_rebuild_avoidance.cmake
```

The first checks the header-guard form, the file naming, the include mechanism, the repository's root
shape and the rule that the core may not name a domain. The second diffs every module's source list
against the files under that module, in both directions, so a file that exists but is built by
nothing — and an entry naming a file that does not exist — stops the configure by name. The third
reports whether the public-surface snapshot still describes the tree. All three run on every
configure of this tree, so a violation stops the build rather than waiting to be asked about. The fourth reads the link graph and asserts what each module may and
may not reach. The fifth extracts the public declarations and compares them against the snapshot in
`tests/golden/`, which is what makes a removal from the shipped surface visible. The sixth builds a
project that consumes praxis, edits a source that project owns, and asserts that the rebuild
recompiles nothing praxis built; it needs Ninja, whose dry run is what that answer is read from.

The last three run as tests rather than at configure: each needs a build tree it drives itself, and
the fourth regenerates the link graph, which is a configure and cannot happen inside one. `ctest`
gives the fourth and the sixth a build directory of their own for that; run by hand each writes under
whatever `PRAXIS_BUILD_DIR` names, which is why the commands above point at scratch paths rather than
at `build`. The sixth is registered only where `PRAXIS_BUILD_CONSUMER_TESTS` asks for it, because it
configures and builds a whole project of its own before it can read anything.

The fifth needs Universal Ctags. Its test is registered whether or not the tool is present and fails
by name where it is absent, because a test that is not registered is silent in the report and a
report that says nothing reads as a report that found nothing. The snapshot it compares against is
only ever written from that extraction: a snapshot assembled by hand cannot be told apart from a real
one, so a machine without the tool cannot regenerate it.

It is the only one of the six that needs the tool, and it is the only one `PRAXIS_ENABLE_CTAGS_GATES`
governs. The option defaults on for this tree and off for a project that adds praxis. Set it off and
every other gate and every unit test still runs — **a machine with no Universal Ctags can run the
suite**, it simply is not checking the shipped surface while it does. Continuous integration is expected to
set it on explicitly, so a machine missing the tool fails there rather than reporting a green nothing.

The first three read this tree rather than a consumer's, so a project that adds praxis runs none of
them and none of the three above. The sixth drives a consuming project, but it is this tree checking
what such a project would rebuild rather than anything that project runs itself.
**A consumer needs no Universal Ctags and runs no gate.**

`CONVENTIONS.md` is the authoritative style specification, `.clang-format` owns the mechanical rules,
and `EXCEPTIONS.md` registers every unit that exceeds a size ceiling with the reason it stays whole.

## Dependencies

Every library this project uses is fetched and built from source by CMake, so almost nothing has to
be installed system-wide. What does have to be present before the first configure:

- A C++20 compiler.
- CMake 3.28 or newer.
- git 2.28 or newer. The resource pipeline uses cone-mode sparse checkouts, which older git cannot
  perform.
- On Linux, the X11 and OpenGL development headers the renderer needs. The renderer names the
  Debian and Ubuntu packages itself in a configure message.

### Where the dependency references live

Every dependency this project fetches is declared in one file, `cmake/dependencies.cmake`, and each
declaration names a fixed tag or a fixed commit. Nothing resolves to a moving branch, so two people
configuring on different days get the same sources.

### Moving a reference deliberately

Ask the remote what a branch or tag currently points at:

```shell
git ls-remote https://github.com/skrede/meios.git develop
```

Then replace that dependency's reference in `cmake/dependencies.cmake` with the hash it prints. The
commit hashes in that file are an interim form — each is expected to become a release tag once the
corresponding library cuts one, so replacing a hash with a tag is the intended direction of travel
and not a deviation.

Changing a reference inside a build directory that already holds a clone of that dependency makes
the update step fail with `Child return code: 128`. That is a stale checkout rather than a bad
reference. Configure into a fresh build directory, or delete `_deps/<name>-src` under the existing
one, and the fetch will resolve the new reference.

### If the KUKA description fails on a hash mismatch

The KUKA description is fetched as an archive pinned by content hash rather than cloned, because
the clone path cannot accept a commit hash and the upstream repository publishes no tag to use
instead. The hosting service generates those archives on demand and has changed their encoding
before. If that happens again every build fails at once, because CMake treats a hash mismatch as
fatal and offers no fallback.

The immediate workaround is to delete the `HASH` line from the `kuka_experimental` declaration in
`cmake/dependencies.cmake` and configure again. The revision stays pinned; only the byte check is
given up.

## License

Apache License 2.0 &mdash; see [LICENSE](LICENSE) for the full text.
