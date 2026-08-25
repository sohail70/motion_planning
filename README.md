# FMT$^{\mathrm{X}}$: Lazy Wavefront Search for Dynamic Replanning

FMT$^{\mathrm{X}}$ is a sampling-based motion replanner that combines lazy,
cost-ordered repair with online graph densification.

[Paper](https://arxiv.org/abs/2509.08521)

<p align="center">
  <img src="./Geometric_S10000_C1_5_I0.gif" alt="Motion planning visualization" width="80%">
</p>

## Planners

- D-FMT$^{*}$
- FMT$^{\mathrm{X}}$
- D$^{*}$ Lite on PRM$^{*}$
- RRT$^{\mathrm{X}}$
- LLPT$^{*}$

Configurations for geometric, R2T, Dubins, and Thruster experiments are under
`config/`. Obstacle environments are under `sims/`.

## Requirements

- ROS 2 Jazzy
- Eigen3
- nanoflann
- yaml-cpp
- tinyxml2

## Build

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

## Run

```bash
./main <config-file.yaml>
```

## Videos

YouTube playlist link will be added here.
