# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS catkin description package for the hexapod/grasp robot model. The root files `package.xml` and `CMakeLists.txt` define package metadata, dependencies, and install rules. Robot descriptions live in `urdf/`, including the main generated URDF, collision URDF, Xacro file, and CAD export CSV. Mesh assets are in `meshes/` as STL files, with optional visual materials in `textures/`. Runtime entry points are in `launch/`: `display.launch` opens the model in RViz and `gazebo.launch` spawns it in Gazebo. Joint-name mapping data is stored in `config/`.

## Build, Test, and Development Commands

Run commands from a catkin workspace that contains this package under `src/`.

```bash
catkin_make
```

Builds and installs package share files declared in `CMakeLists.txt`.

```bash
source devel/setup.bash
roslaunch 抓取机器人export_urdf.SLDASM display.launch
roslaunch 抓取机器人export_urdf.SLDASM gazebo.launch
```

Loads the robot in RViz or Gazebo. Use shell tab completion for the package name to avoid Unicode typos.

```bash
roslaunch-check launch/display.launch
roslaunch-check launch/gazebo.launch
```

Checks launch-file syntax and package references when `roslaunch` tools are available.

## Coding Style & Naming Conventions

Keep XML files two-space indented and preserve generated URDF link, joint, and mesh names unless regenerating the whole export. Prefer lowercase directory names matching ROS conventions: `config/`, `launch/`, `meshes/`, and `urdf/`. For new launch files, use descriptive names such as `display_collision.launch` or `gazebo_empty_world.launch`. Avoid renaming the package without updating every `$(find ...)` reference.

## Testing Guidelines

There is no dedicated test suite in this checkout. Validate changes by building with `catkin_make`, running `roslaunch-check`, and opening both RViz and Gazebo launch files. For URDF edits, also run:

```bash
check_urdf urdf/抓取机器人export_urdf.SLDASM.urdf
```

Confirm that all referenced mesh paths resolve and that joint names match `config/joint_names_抓取机器人export_urdf.SLDASM.yaml`.

## Commit & Pull Request Guidelines

This checkout has no `.git` history, so no project-specific commit convention can be inferred. Use short imperative commits, for example `Fix Gazebo spawn model path` or `Update knee link collision mesh`. Pull requests should describe the affected robot files, list validation commands run, and include screenshots or short notes for RViz/Gazebo visual changes. Link related issues when available and call out regenerated CAD exports explicitly.

## Agent-Specific Instructions

Treat generated CAD/URDF artifacts carefully. Keep edits focused, avoid broad formatting churn in generated files, and document any manual changes that would need to be repeated after a future export.
