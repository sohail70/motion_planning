# A Motion Planning Library Featuring the FMTX Algorithm

This repository contains a C++ library for advanced motion planning, created to develop and showcase my **FMTX** algorithm. FMTX is a sampling-based replanning method, derived from FMT*, designed for efficient navigation in dynamic environments. The initial concept for `FMTX` emerged during my studies while implementing Michael Otte's `RRTX` algorithm. I first explored `FMTX` in MATLAB, but I encountered challenges with FMT*'s inherent constraints. After some time, I revisited the problem and developed the complete solution presented in this library.

For those interested in the development process, the following links contain my exploratory MATLAB and Python files. Please note that these are development archives; the code is not organized and contains many experimental ideas for adapting FMT* to handle dynamic obstacles. The finalized, working solution is in this C++ repository.

* **Initial Concept in MATLAB/Python**: [Development Files](https://drive.google.com/drive/folders/1BwNw0cQw3J7h2tKtNB-NLXXTFCvpezjT?usp=drive_link)
* **Original `RRTX` Implementation**: [Student Project Files](https://drive.google.com/drive/folders/1K904_q35ITkBdSvvj18UvrUiA0Kab96B?usp=drive_link)

While the primary focus is `FMTX`, the library expanded to include implementations of several official static planners. This provides a baseline for future work and serves as a personal testbed for exploring other experimental concepts, such as my unofficial algorithms `AnyFMT` and `InformedAnyFMT`, which are not based on existing literature.

The plots and tables generated for the associated research paper can be found here:
* **Research Paper Assets**: [Plots and Tables](https://drive.google.com/drive/folders/1U5jQpnKM32EV4fS4kByw6YucyPG08OkS?usp=drive_link)

---

## Key Features

* **Algorithms**:
    * **Geometric Planners**: For pathfinding in configuration space, ignoring time and dynamics.
        * **FMT\* and Variants**: Fast Marching Tree (`FMT*`), Anytime FMT\* (`AnyFMT`), and FMT\* with an A* heuristic (`FMTA`), along with informed versions (`InformedAnyFMT`, `InformedAnyFMTA`).
          > **Note**: These anytime variants are my own unofficial concepts. While some papers have explored making `FMT*` an anytime planner, they often use incremental rewiring, which differs from the `FMTX` approach. I am open to discussion on this topic. I plan to explore these ideas more formally in the future.
        * **BIT\***: Batch Informed Trees, implemented to benchmark my single-queue concepts against its dual-queue structure.
        * **Replanning Algorithms**: `FMTX` (my novel algorithm) and `RRTX` for robust navigation in dynamic environments.
    * **Kinodynamic Planners**: For state spaces where dynamics and time are critical.
        * **Kinodynamic FMTX & RRTX**: Versions of the replanners tailored for complex state spaces like R2T (position + time), Dubins (car-like), and Thruster (position + velocity + time).

* **Advanced Data Structures**:
    * **NanoFLANN**: For fast nearest-neighbor searches.
    * **Weighted NanoFLANN**: To prioritize certain state-space dimensions.
    * **Lie Group Splitting KD-Tree**: An experimental feature that currently has performance limitations.

* **ROS 2 and Gazebo Integration**:
    * Seamlessly integrates with ROS 2 for visualization (`RViz`) and communication.
    * Includes a custom Gazebo-based obstacle checker (`gz_obs`) for realistic, physics-based collision detection.

* **Modular and Extensible**: The library is built with a modular design, making it easy to add and test new planners, state spaces, and robot models in dynamic environments. As a solo project, there are still many improvements and features I plan to add over time.

---

## 🚀 Getting Started

### Dependencies

Before building, ensure you have the following dependencies installed. While `Bullet` and `FCL` are included for testing, I've found my custom obstacle checker to be faster for my use cases. Mind that I built the library in ROS2 Jazzy.

* **Core Libraries**:
    * `Eigen3`
    * `nanoflann`
    * `OpenMP`
    * `Bullet`
    * `FCL`
    * `ZeroMQ` & `Protobuf`
    * `ROS2`

### Building and Running

This project uses a standard `CMake` build process.

**1. Build the Main Library**
```bash
# Clone the repository
git clone https://github.com/sohail70/motion_planning
cd motion_planning

# Create a build directory and build the library
mkdir build && cd build
cmake ..
make
```

**2. Build the Gazebo Plugins**
The Gazebo plugins for controlling obstacles are in the `sim` directory and must be built separately.

```bash
# From the root of the repository
cd sim

# Create a build directory and build the plugins
mkdir build && cd build
cmake ..
make
```

**3. Run a Simulation**
After building, you can run the planners. Open three separate terminals.

* **Terminal 1: Start the ROS-Gazebo Bridge** (Provides the simulation clock to ROS)
    ```bash
    # From the root of the repository
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./sim/ros_gz_bridge.yaml
    ```

* **Terminal 2: Launch Gazebo**
    * For **geometric** tests with complex obstacle motion:
        ```bash
        # From the root of the repository
        gz sim -s sim/worlds/dynamic_worlds.sdf
        ```
    * For **kinodynamic** tests with constant velocity obstacles:
        ```bash
        # From the root of the repository
        gz sim -s sim/worlds/<kinodynamic_world_name>.sdf
        ```

* **Terminal 3: Launch RViz**
    ```bash
    # From the root of the repository
    rviz2 -d ./sim/default.rviz
    ```

* **Terminal 4: Run a Planner Test**
    > **Important**: The test executables currently have hardcoded paths to the SDF world files for parsing obstacle dimensions. You will need to update these paths inside the test files to match your system. I plan to move these parameters to a YAML file in the future.

    ```bash
    # From the motion_planning/build directory
    # Geometric Examples:
    ./test_rrtx --samples 1000 --factor 1.5 --seed 42 --duration 30
    ./test_fmtx --samples 1000 --factor 1.5 --seed 42 --duration 30

    # Kinodynamic Examples:
    ./test_kinodynamic_fmtx_R2T --samples 5000 --factor 2.5 --seed 42
    ./test_kinodynamic_fmtx_dubin_4D --samples 3000 --factor 2.5 --seed 42
    ./test_kinodynamic_fmtx_thruster_5D --samples 1000 --factor 2.5 --seed 42

    ./test_kinodynamic_rrtx_R2T --samples 5000 --factor 2.5 --seed 42
    ./test_kinodynamic_rrtx_dubin_4D --samples 3000 --factor 2.5 --seed 42
    ./test_kinodynamic_rrtx_thruster_5D --samples 1000 --factor 2.5 --seed 42
    ```

### A Note on `FMT*` vs. `RRT*` Based Planners
An interesting insight from this work is the "weighted trade-off" between `FMT*` and `RRT*` based algorithms. `FMT*` variants tend to have more neighbor processing loops during their Bellman updates but perform fewer obstacle checks. In contrast, `RRT*` variants perform more obstacle checks but have less intensive rewiring cascades. A key feature of `FMTX` is the ability to use a **k-NN** search for neighbors instead of a fixed radius, which makes the number of neighbors more predictable and can significantly speed up the planner. `RRTX`, due to its `cullNeighbors` function, is architecturally bound to a radius search.

---

### Demos
Here are some visualizations of the planners in action:

**`./test_fmtx --samples 1000 --factor 1.5 --seed 42 --duration 30`** (Zero inflation)
<p align="center">
  <img src="./Geometric_S10000_C1_5_I0.gif" alt="Geometric_S10000_C1_5_I0">
</p>

**`./test_kinodynamic_fmtx_R2T --samples 5000 --factor 2.5 --seed 42`** (0.5m inflation)
<p align="center">
  <img src="./R2T_S5000_C2_5_I0_5.gif" alt="R2T_S5000_C2_5_I0_5">
</p>

**`./test_kinodynamic_fmtx_dubin_4D --samples 3000 --factor 2.5 --seed 42`**
<p align="center">
  <img src="./Dubins_S3000_C2_5_I2_0.gif" alt="Dubins_S3000_C2_5_I2_0">
</p>

**`./test_kinodynamic_fmtx_thruster_5D --samples 1000 --factor 2.5 --seed 42`**
<p align="center">
  <img src="./Thruster_S1000_C2_5_I0_5.gif" alt="Thruster_S1000_C2_5_I0_5">
</p>

**`./test_kinodynamic_fmtx_thruster_5D --samples 2000 --factor 2.5 --seed 42`**
<p align="center">
  <img src="./Thruster_S2000_C2_5_I0_5.gif" alt="Thruster_S2000_C2_5_I0_5">
</p>

**`./test_kinodynamic_fmtx_thruster_5D --samples 1000 --factor 3.0 --seed 42`**
<p align="center">
  <img src="./Thruster_S1000_C3_0_I0_5.gif" alt="Thruster_S1000_C3_0_I0_5">
</p>