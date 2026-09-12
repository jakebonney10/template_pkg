# ROS2 Template C++ Package
![alt text](docs/media/template_package.png)
This package was designed as a starting point for building a ROS2 C++ package with:
- automatically configuring cmake file
- pre-defined structures and conventions for namespacing and code structures
- adherence to ROS2 design conventions
- pre-included doxygen configuration with easy to use and stylish output
- pre-defined github actions to build the documentation and deploy it to github.io

It is highly recommended that you complete the following checklist before you make public a repository derived from this package.

# Public Repository Checklist   

## Requirements

All public repositories for this organization must comply with the following checklist.  If they do not, an administrator will make your repository private until it complies. 

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

- **Adherence to our [Style Guide](style_guide):** This is especially recommended if you are starting a new project.  If you are migrating an old one, it can be overlooked.
- **Use of GitFlow:** Follow our [guidelines on version control](version_control) for more info

## Suggested
- Issues template
- Doxygen documentation for C++ code (a template can be found in this package)



# README template  

A few lines describing what your project is and what it does.   A picture of your software in action is highly recommended.

## Installation

how to install the package without compilation.

## Compiling

how to compile the package including dependencies and nicely formatted commands.  This section should be geared toward developers and collaborators.

```
sudo make install
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
