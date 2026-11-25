# Contributing

## Branching Strategy

This repository follows the GitFlow Workflow.  If you are not familiar, please review [this article](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).  Branch names, prefixes and version numbering are described in our [version control guidelines](https://bonrov.github.io/development-conventions/version_control.html).

## Think Before you Fork

Before you create a fork, it is advisable to create an issue on the github page.  That way, the maintainers can let you know if that feature or bugfix is already under development.

Please consult the maintainers before you create a fork that will change existing behaviors.

## Fixing a Bug in Production Code

To fix a bug make a create a branch from `master`. This branch should be labeled `hotfix/x.y.z-description` where x.y.z follow the [semantic versioning convention](https://semver.org/).

When the bugfix is complete, make a pull request back to `master` and the maintainers will review and merge it.  

## Adding a Feature

If you wish to add a new feature, please make a fork from the `devel` branch.

Once your modification is completed, make a pull request back to the `devel` branch.  If it is accepted, your modifications will be available in the `master` branch (default branch) after the next release.

## Code Style

All code follows the [ROS2 code style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-to-code/Code-Style-Language-Versions.html) and must pass the standard ROS2 linters (`ament_lint_common`).  The GitHub build checks run them on every pull request.  Before you open one, run them yourself from the root of your workspace:

```bash
colcon test
colcon test-result --verbose
```

`ament_uncrustify --reformat <path>` fixes most C++ formatting problems automatically.

## Additional Guidelines

Please review and adhere to our [Development Conventions](https://bonrov.github.io/development-conventions/), in particular the [Style Guide](https://bonrov.github.io/development-conventions/style_guide.html).
