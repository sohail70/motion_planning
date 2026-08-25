# FMT$^{\mathrm{X}}$: Lazy Wavefront Search for Dynamic Replanning

This repository contains the C++17 implementation of FMT$^{\mathrm{X}}$, a
sampling-based planner for dynamic replanning that extends the lazy,
cost-ordered wavefront of FMT$^{*}$. The implementation supports fixed-graph
repair through Dynamic FMT$^{*}$ (D-FMT$^{*}$) and anytime replanning through
online densification in FMT$^{\mathrm{X}}$.

Paper: [FMT$^{\mathrm{X}}$: Lazy Wavefront Search for Dynamic Replanning](https://arxiv.org/abs/2509.08521)

<p align="center">
  <img src="./Geometric_S10000_C1_5_I0.gif" alt="Motion planning visualization" width="80%">
</p>

## Overview

The academic experiments use deterministic, event-driven obstacle updates and
exact collision checking. ROS 2 and RViz are retained for visualization and
simulated robot motion management. Gazebo-based perception and Kalman-filtered
obstacle estimation are not part of the reported evaluation.

The implementation includes the planners used in the paper:

- D-FMT$^{*}$ on a fixed sampled graph
- FMT$^{\mathrm{X}}$ with online densification
- D$^{*}$ Lite on a fixed PRM$^{*}$ graph
- RRT$^{\mathrm{X}}$
- LLPT$^{*}$

The benchmarks cover geometric $\mathbb{R}^{2}$ planning and the R2T, Dubins,
and Thruster state spaces.

## Repository Scope

The unified academic benchmark is implemented in
`test/unified/main.cpp` and configured through YAML files under `config/`.
Older geometric planners and unit tests are retained as library components but
are not part of the reported T-RO evaluation.

The main configuration groups are:

| Directory | Planner |
| --- | --- |
| `config/fmtx/` | Fixed-graph D-FMT$^{*}$ |
| `config/anyfmtx/` | Anytime FMT$^{\mathrm{X}}$ |
| `config/prmstar_dstarlite/` | D$^{*}$ Lite on PRM$^{*}$ |
| `config/rrtx/` | Fixed-sample RRT$^{\mathrm{X}}$ configurations |
| `config/anyrrtx/` | Anytime RRT$^{\mathrm{X}}$ |
| `config/llpt_star/` | LLPT$^{*}$ |

## Dependencies

The current CMake configuration requires:

- CMake 3.10 or newer and a C++17 compiler
- ROS 2 with `ament_cmake`, `rclcpp`, `visualization_msgs`, and `tf2_ros`
- Eigen3
- nanoflann
- yaml-cpp
- tinyxml2
- qpOASES
- Abseil

The project was developed and evaluated with ROS 2 Jazzy on Ubuntu 24.04.

## Build

Source the ROS 2 environment and build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

## Run

The unified executable takes one YAML configuration file:

```bash
./main <config_file.yaml>
```

Run it from the `build` directory so that the relative SDF paths in the supplied
configurations resolve correctly. For example:

```bash
./main ../config/anyfmtx/r2.yaml
./main ../config/anyfmtx/dubins.yaml
./main ../config/anyfmtx/thruster.yaml
./main ../config/prmstar_dstarlite/r2.yaml
./main ../config/llpt_star/dubins.yaml
```

Although the YAML field containing the environment path is currently named
`gazebo_params.sdf_path`, the academic runner parses the SDF directly and uses
the deterministic obstacle checker. A Gazebo process is not required.

RViz is optional for viewing planner state and robot motion:

```bash
rviz2 -d ../sim/default.rviz
```

## Demo Videos

YouTube playlist link will be added here.

## Reproducing Paper Experiments

Each experiment is defined by its YAML configuration, including the planner,
state space, random seed, sampling parameters, robot constraints, obstacle
environment, and simulation timing. Use the corresponding configuration under
`config/` with the unified `main` executable. The runner records replanning and
trajectory metrics for subsequent aggregation and plotting.

For the precise evaluation protocol, statistical aggregation, and reported
comparisons, refer to the [paper](https://arxiv.org/abs/2509.08521).
