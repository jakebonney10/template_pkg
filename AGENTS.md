# Agent instructions

Instructions for AI coding agents working in this repository.  It is a
template for ROS2 C++ packages; see [README.md](README.md) for the overview.

## Layout

- `template_pkg/` - the C++ package (`nodes/` holds one `main()` per
  executable, `src/` and `include/template_pkg/` hold the node classes,
  plus `launch/` and `config/`).
- `template_pkg_interfaces/` - message definitions.
- `docs/` - Doxygen configuration and extra documentation pages.
- `.github/workflows/` - build checks, documentation deploy, releases.

## Build, lint and test

Run these from the root of a colcon workspace that has this repository in
`src/`:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test
colcon test-result --verbose
```

`colcon test` runs the standard ROS2 linters (`ament_lint_common`).  A change
is not finished until it builds and `colcon test-result` reports no failures.
The GitHub build checks run the same thing on Humble, Jazzy and Lyrical, so
code must work on all three.

## Code style

Follow the [ROS2 code style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-to-code/Code-Style-Language-Versions.html).
The linters enforce it; do not disable or exclude a linter to make a check
pass.  The organization's additions to that style, and its version control
and release rules, are in the
[BonROV Development Conventions](https://bonrov.github.io/development-conventions/).

- C++: 2-space indentation, lines of at most 100 characters, opening braces
  of functions, classes and namespaces on their own line.  Run
  `ament_uncrustify --reformat <path>` to fix formatting.  Use the Humble
  version when the distributions disagree, because it is the strictest.
- Headers start with `#pragma once`.
- Types are `CamelCase`; functions, methods and variables are `snake_case`;
  member variables end with an underscore.
- Include order: the file's own header, C++ standard library, then other
  packages.  Include with the package prefix, e.g.
  `#include "template_pkg/minimal_publisher_node.hpp"`.
- All C++ code lives in a namespace named after the package, opened and
  closed with the `NS_HEAD` / `NS_FOOT` macros from
  `template_pkg/package_defs.hpp`.  Keep each macro definition on a single
  line inside its `INDENT-OFF` block, or the formatter cannot parse the file.
- Every C++ and Python source file starts with the license header.  Add
  missing ones with `ament_copyright --add-missing "<owner>" mit <path>`.
- Python (launch files): flake8 and pep257 clean, with a blank line between
  standard library and third-party imports.
- `package.xml` must validate against the format 3 schema, which fixes the
  order of its elements (`test_depend` before `member_of_group` and `export`).

## CMake

- Add ROS dependencies to the `ROS_DEPENDS` list in
  `template_pkg/CMakeLists.txt` and as `<depend>` in `package.xml`.
- Do not use `ament_target_dependencies()`; it was removed in Lyrical.  The
  CMake file links the imported targets of everything in `ROS_DEPENDS`.
- Every `.cpp` file in `nodes/` becomes an executable automatically.

## Node structure

Nodes follow the pattern in `MinimalPublisherNode`: nested `Parameters`,
`Subscribers`, `Publishers` and `Diagnostics` structs, each with its own
`declare`/`update` or `init` method called from the constructor.  Keep
parameter defaults in the header and in `config/*.yaml` in sync.

## Releases

Each package keeps a `CHANGELOG.rst` in REP-132 format; add user-visible
changes to its `Forthcoming` section.  Both `package.xml` files must carry
the same `<version>`.  A release is made
by pushing a `vX.Y.Z` tag that matches it; the release workflow fails
otherwise.  Do not create or push tags unless asked.
