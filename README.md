# Dynamic Motion Planning & Replanning

A C++17 and ROS 2 research library for sampling-based motion planning and
online replanning in dynamic environments. The repository provides a shared
experimental framework for geometric and kinodynamic planners, deterministic
obstacle motion, and RViz visualization.

[![arXiv](https://img.shields.io/badge/arXiv-2509.08521-b31b1b.svg)](https://arxiv.org/abs/2509.08521)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

<p align="center">
  <img src="./Geometric_S10000_C1_5_I0.gif" alt="Dynamic motion replanning visualization" width="60%">
</p>

## Video Demonstrations

<p align="center">
  <a href="https://www.youtube.com/watch?v=HMxyYnfQoOE&list=PLIZXjL-DM2LQ&pp=sAgC">
    <strong>▶ Watch the FMTX Dynamic Replanning Demonstrations on YouTube</strong>
  </a>
</p>

## Paper

This repository accompanies:

[**FMT<sup>X</sup>: Lazy Wavefront Search for Dynamic Replanning**](https://arxiv.org/abs/2509.08521)

Soheil Espahbodi Nia, arXiv:2509.08521.

FMT<sup>X</sup> combines lazy, cost-ordered graph repair with online graph
densification for motion replanning in changing environments.

## Planners

The unified experimental runner includes the principal dynamic replanners used
in the paper:

- D-FMT<sup>*</sup>
- FMT<sup>X</sup>
- D<sup>*</sup> Lite on PRM<sup>*</sup>
- RRT<sup>X</sup>
- LLPT<sup>*</sup>

Experiment configurations cover geometric, R2T, Dubins, and Thruster state
spaces. Planner configurations are under [`config/`](config/), and obstacle
environments are under [`sims/`](sims/).

## Requirements

- A C++17 compiler
- CMake 3.10 or newer
- ROS 2 Jazzy
- `ament_cmake`
- `rclcpp`, `visualization_msgs`, and `tf2_ros`
- Eigen3
- nanoflann
- yaml-cpp
- tinyxml2

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build
cmake --build build -j
```

## Run an experiment

Run the executable from the `build/` directory. The SDF paths in the current
experiment configurations are relative to that directory.

```bash
cd build
./main ../config/anyfmtx/r2t.yaml
```

To select another planner or state space, pass a different YAML configuration:

```bash
./main ../config/rrtx/dubins.yaml
./main ../config/llpt_star/thruster.yaml
./main ../config/prmstar_dstarlite/r2.yaml
```

## Repository layout

| Path | Description |
| --- | --- |
| [`include/motion_planning/`](include/motion_planning/) | Public headers and planner interfaces |
| [`src/`](src/) | Planner, state-space, and utility implementations |
| [`test/unified/main.cpp`](test/unified/main.cpp) | Unified experiment runner |
| [`config/`](config/) | Planner and state-space configurations |
| [`sims/`](sims/) | Static and dynamic obstacle environments |
| [`scripts/`](scripts/) | Experiment and plotting utilities |

## Citation

If you use this repository in academic work, please cite:

```bibtex
@misc{espahbodinia2025fmtx,
  title         = {FMT$^{\mathrm{X}}$: Lazy Wavefront Search for Dynamic Replanning},
  author        = {Espahbodi Nia, Soheil},
  year          = {2025},
  eprint        = {2509.08521},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO}
}
```

## License

This project is available under the [MIT License](LICENSE).
