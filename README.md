# ROS2 Template C++ Package

[![Humble](https://github.com/jakebonney10/template_pkg/actions/workflows/humble.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/humble.yaml)
[![Jazzy](https://github.com/jakebonney10/template_pkg/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/jazzy.yaml)
[![Lyrical](https://github.com/jakebonney10/template_pkg/actions/workflows/lyrical.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/lyrical.yaml)
[![Pages](https://github.com/jakebonney10/template_pkg/actions/workflows/pages.yml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/pages.yml)

![alt text](docs/media/template_package.png)

This package was designed as a starting point for building a ROS2 C++ package with:
- automatically configuring cmake file
- pre-defined structures and conventions for namespacing and code structures
- adherence to ROS2 design and code style conventions, enforced by the standard ROS2 linters
- pre-included doxygen configuration with easy to use and stylish output
- full-text search of the documentation (press Cmd/Ctrl+K on the documentation site)
- an example node with YAML-configurable topics and QoS, plus `diagnostic_updater` support
- a separate interfaces package (`template_pkg_interfaces`) for custom messages
- pre-defined github actions that build and test the packages on every supported ROS2 distribution
- pre-defined github actions to build the documentation and deploy it to github.io

## Supported ROS2 distributions

| Distribution | Ubuntu | Build status |
| --- | --- | --- |
| Humble Hawksbill | 22.04 | [![Humble](https://github.com/jakebonney10/template_pkg/actions/workflows/humble.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/humble.yaml) |
| Jazzy Jalisco | 24.04 | [![Jazzy](https://github.com/jakebonney10/template_pkg/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/jazzy.yaml) |
| Lyrical Luth | 26.04 | [![Lyrical](https://github.com/jakebonney10/template_pkg/actions/workflows/lyrical.yaml/badge.svg)](https://github.com/jakebonney10/template_pkg/actions/workflows/lyrical.yaml) |

## Repository layout

| Path | Purpose |
| --- | --- |
| `template_pkg/` | The C++ package: node sources, headers, launch and config files |
| `template_pkg_interfaces/` | Message definitions used by `template_pkg` |
| `docs/` | Doxygen configuration, theme, and extra documentation pages |
| `.github/workflows/` | Build checks (one per ROS2 distribution) and documentation deployment |

## GitHub Actions

| Workflow | Runs on | What it does |
| --- | --- | --- |
| `humble.yaml`, `jazzy.yaml`, `lyrical.yaml` | pushes to `master`/`main`/`devel`, every pull request, manual dispatch | Uses [industrial_ci](https://github.com/ros-industrial/industrial_ci) to install dependencies with `rosdep`, build every package in the repository, and run its tests in a clean container for that distribution |
| `pages.yml` | pushes to `master`/`main`, manual dispatch | Builds the Doxygen documentation, stamps it with the latest release tag, builds the full-text search index with [Pagefind](https://pagefind.app), and deploys it to GitHub Pages |
| `release.yaml` | pushing a `vX.Y.Z` tag | Checks that the tag matches the `<version>` in every `package.xml`, then creates a GitHub Release with generated release notes |

### Making a release

1. In each package's `CHANGELOG.rst`, rename the `Forthcoming` section to the new version and date.
2. Set the same new `<version>` in `template_pkg/package.xml` and `template_pkg_interfaces/package.xml`, and merge that to `master`.
3. Tag that commit and push the tag:

```bash
git tag v1.2.3
git push origin v1.2.3
```

The release workflow fails without publishing anything if the tag and the package versions disagree.

Both packages enable the standard ROS2 linters (`ament_lint_common`), so the build checks fail if the code does not follow the [ROS2 code style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-to-code/Code-Style-Language-Versions.html). Any test you register in a package's `CMakeLists.txt` (see the commented `ament_cmake_gtest` example in `template_pkg/CMakeLists.txt`) is run as well.

### Checking your code locally

Run the same linters and tests the build checks run:

```bash
colcon test
colcon test-result --verbose
```

Fix C++ formatting automatically, and add missing license headers:

```bash
ament_uncrustify --reformat src/template_pkg
ament_copyright --add-missing "Your Name" mit src/template_pkg
```

When you derive a repository from this template:

1. Replace the license header at the top of every source file, and the `@todo` license and maintainer in each `package.xml`, with your own.
1. Replace `jakebonney10/template_pkg` in the badge URLs at the top of this file with your own `owner/repository`.
2. To support another distribution, copy one of the distribution workflows and change its `name` and `ROS_DISTRO`. To drop one, delete its workflow file and badge.
3. Enable GitHub Pages for the repository (**Settings → Pages → Source: GitHub Actions**) so `pages.yml` can deploy.
4. Optionally require the build checks before merging (**Settings → Branches → Branch protection rules**).

The template is the working example for the [BonROV Development Conventions](https://bonrov.github.io/development-conventions/), which describe the style, version control and publication rules it follows.

It is highly recommended that you complete the following checklist before you make public a repository derived from this package.

# Public Repository Checklist   

## Requirements

All public repositories for this organization must comply with the following checklist, which comes from the [BonROV Development Conventions](https://bonrov.github.io/development-conventions/repository_publication_guidelines.html).  If they do not, an administrator will make your repository private until it complies. 

- **README.md file:** Every package must have a README.md file in the root of the repository.  The readme must describe, at least, the following:
  - The name of the package
  - A few sentences briefly describing what the package is for
  - Installation instructions detailed enough to be executed by a novice linux/ROS user
  - A quick start guide or hello world
  - A template [README.md](README.md) is available below.
- **CONTRIBUTING.md file:** This file describes how to contribute to the project.  A default [CONTRIBUTING.md](CONTRIBUTING.md) is available here.  It can be modified as necessary.
- **LICENSE file:** A license file should be included. (the licence file in this packag has some options)
- **Semantic Versioning:** The repo must adhere to [Semantic Versioning 2.0](https://semver.org/).
- **An initial tagged release:** 
  - If the project is ready to ship  include initial release with `v1.0.0`
  - Pre-release code may be shared and must be tagged as `v0.x.y`
- **All code in the master branch must ALWAYS be deployable**

## Highly Encouraged

- **Adherence to our [Style Guide](https://bonrov.github.io/development-conventions/style_guide.html):** This is especially recommended if you are starting a new project.  If you are migrating an old one, it can be overlooked.
- **Use of GitFlow:** Follow our [guidelines on version control](https://bonrov.github.io/development-conventions/version_control.html) for more info

## Suggested
- Issues template
- Doxygen documentation for C++ code (a template can be found in this package)



# README template  

A few lines describing what your project is and what it does.   A picture of your software in action is highly recommended.

## Installation

how to install the package without compilation.

## Compiling

how to compile the package including dependencies and nicely formatted commands.  This section should be geared toward developers and collaborators.

For this template, clone the repository into the `src` directory of a colcon workspace, install the dependencies with `rosdep`, and build:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/jakebonney10/template_pkg.git
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Running the package

A basic hello world use of your package

## Parameters

The example node reads its topic names, timer period, and publisher/subscriber
QoS profiles from `template_pkg/config/minimal_publisher.yaml`. Publisher and
subscriber QoS can be configured independently without recompiling:

```yaml
minimal_publisher:
  ros__parameters:
    qos:
      subscriber:
        reliability: best_effort
        durability: volatile
        history: keep_last
        depth: 10
      publisher:
        reliability: best_effort
        durability: volatile
        history: keep_last
        depth: 10
```

The supported values are:

| Parameter | Values |
| --- | --- |
| `reliability` | `best_effort`, `reliable`, `system_default` |
| `durability` | `volatile`, `transient_local`, `system_default` |
| `history` | `keep_last`, `keep_all` |
| `depth` | Any positive integer (used by `keep_last`) |

`best_effort` is the default for this template and is often appropriate for
high-rate data where the newest sample matters more than retransmitting an old
one. Use `reliable` when every message must be delivered. A publisher and
subscriber must have compatible QoS policies before ROS 2 will connect them.

Launch the configured example with:

```bash
ros2 launch template_pkg minimal_publisher.launch.py
```

## Services

a description of any ros services your package uses

## Contributing

Thank you for considering a contribution to this package.   Please review our [CONTRIBUTING](CONTRIBUTING.md) guidelines to get started.
