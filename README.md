# A Motion Planning Library Featuring the FMTX Algorithm

This repository contains a C++ library for advanced motion planning, created to develop and showcase my **FMTX** algorithm. FMTX is a sampling-based replanning method, derived from FMT*, designed for efficient navigation in dynamic environments. The initial concept for `FMTX` emerged during my studies while implementing Michael Otte's `RRTX` algorithm. I first explored `FMTX` in MATLAB, but I encountered challenges with FMT*'s inherent constraints. After some time, I revisited the problem and developed the complete solution presented in this library.

For those interested in the development process, the following links contain my exploratory MATLAB and Python files. Please note that these are development archives; the code is not organized and contains many experimental ideas for adapting FMT* to handle dynamic obstacles. The finalized, working solution is in this C++ repository.

* **Initial Concept in MATLAB/Python**: [Development Files](https://drive.google.com/drive/folders/1BwNw0cQw3J7h2tKtNB-NLXXTFCvpezjT?usp=drive_link)
* **My `RRTX` Implementation**: [Student Project Files](https://drive.google.com/drive/folders/1K904_q35ITkBdSvvj18UvrUiA0Kab96B?usp=drive_link), [ROS1](https://github.com/sohail70/RRTx), [Phase1](https://drive.google.com/drive/folders/1vjbUe3UOlOzak5UT--D6IquYbJbunH73?usp=drive_link), [Phase2](https://drive.google.com/drive/folders/1zIe1BoMTaTUWE20KnGT3E-d0ywzwD__t?usp=drive_link)

While the primary focus is `FMTX`, the library expanded to include implementations of several official static planners. This provides a baseline for future work and serves as a personal testbed for exploring other experimental concepts, such as my unofficial algorithms `AnyFMT` and `InformedAnyFMT`, which are not based on existing literature.

The data with which I created the plots and tables for the associated research paper can be found here:
* **Research Paper Assets**: [Data](https://drive.google.com/drive/folders/1U5jQpnKM32EV4fS4kByw6YucyPG08OkS?usp=drive_link)

---

## Key Features

* **Algorithms**:
    * **Geometric Planners**: For pathfinding in configuration space, ignoring time and dynamics.
        * **FMT\* and Variants**: Fast Marching Tree (`FMT*`), Anytime FMT\* (`AnyFMT`), and FMT\* with an A* heuristic (`FMTA`), along with informed versions (`InformedAnyFMT`, `InformedAnyFMTA`).
          > **Note**: These anytime variants are my own unofficial concepts. While some papers have explored making `FMT*` an anytime planner, they often use incremental rewiring, which differs from the `FMTX` approach. I am open to discussion on this topic. I plan to explore these ideas more formally in the future.
        * **BIT\***: Batch Informed Trees, implemented to benchmark my single-queue concepts against its dual-queue structure.
        * **Replanning Algorithms**: `FMTX` (my algorithm) and `RRTX` for navigation in dynamic environments.
    * **Kinodynamic Planners**: For state spaces where dynamics and time are critical.
        * **Kinodynamic FMTX & RRTX**: Versions of the replanners tailored for complex state spaces like R2T (position + time), Dubins (car-like), and Thruster (position + velocity + time), with the eventual goal of integrating this library into PX4 flight control simulations.

* **Advanced Data Structures**:
    * **NanoFLANN**: For fast nearest-neighbor searches.
    * **Weighted NanoFLANN**: To prioritize certain state-space dimensions.

* **ROS 2 and Gazebo Integration**:
    * integrated with ROS 2 for visualization (`RViz`) and communication and collision detection.

* **Modular and Extensible**: The library is built with a modular design, making it easy to add and test new planners, state spaces, and robot models in dynamic environments. As a solo project, there are still many improvements and features I plan to add over time.

---

## Getting Started

### Dependencies

Before building, ensure you have the following dependencies installed. While `Bullet` and `FCL` are included for testing, I've found my custom obstacle checker to be faster for my use cases. Mind that I built the library in ROS2 Jazzy.

* **Core Libraries**:
    * `Eigen3`
    * `nanoflann`
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
After building, you can run the planners. Open four separate terminals.

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
        gz sim -s sim/dynamic_world_straight_box_circle_10.sdf
        ```

* **Terminal 3: Launch RViz**
    ```bash
    # From the root of the repository
    rviz2 -d ./sim/default.rviz
    ```

* **Terminal 4: Run a Planner Test**
    > **Important**: The test executables currently have hardcoded paths to the SDF world files for parsing obstacle dimensions. You will need to update these paths inside the test files to match your system. I plan to move these and other parameters to a YAML file in the future.

    ```bash
    # From the motion_planning/build directory
    # Geometric Examples:
    ./test_rrtx --samples 10000 --factor 1.5 --seed 42 --duration 30
    ./test_fmtx --samples 10000 --factor 1.5 --seed 42 --duration 30

    # Kinodynamic Examples:
    ./test_kinodynamic_fmtx_R2T --samples 5000 --factor 2.5 --seed 42
    ./test_kinodynamic_fmtx_dubin_4D --samples 3000 --factor 2.5 --seed 42
    ./test_kinodynamic_fmtx_thruster_5D --samples 1000 --factor 2.5 --seed 42

    ./test_kinodynamic_rrtx_R2T --samples 5000 --factor 2.5 --seed 42
    ./test_kinodynamic_rrtx_dubin_4D --samples 3000 --factor 2.5 --seed 42
    ./test_kinodynamic_rrtx_thruster_5D --samples 1000 --factor 2.5 --seed 42
    ```

### A Note on Algorithmic Trade-Offs

An interesting insight from this work is the fundamental trade-off between `FMT*`-based and `RRT*`-based algorithms. Here's a breakdown of their different approaches to computational challenges:

* **Computational Strategy**: `FMT*` variants tend to perform more intensive neighbor processing during their Bellman updates but execute fewer collision checks. In contrast, `RRT*` variants perform more frequent collision checks but have less computationally heavy rewiring cascades.

* **Neighbor Search**: A key feature of `FMTX` is its ability to use a **k-NN** search for finding neighbors. This makes the size of the neighbor set more predictable and can speed up the planner, especially in dense regions. `RRTX`, on the other hand, is architecturally bound to a radius search due to its `cullNeighbors` function.

* **Obstacle Checking**: The two algorithms also differ in their collision-checking philosophy. `RRTX` uses a **proactive** check, evaluating all edges in the vicinity of an obstacle change. `FMTX` defaults to a **lazy** (or delayed) check, only evaluating an edge's validity when it is a candidate for the optimal path. However, due to the flexible architecture of `FMTX`, I have also implemented a proactive mode. You can switch between these behaviors using the `mode` parameter (`mode 1` for lazy, `mode 2` for proactive).

* **Performance Optimizations**: `FMTX` also includes other features like **obstacle caching**, where collision information is cached and reused within a single planning cycle to avoid redundant checks.

It's also important to acknowledge the inherent limitations of any replanning algorithm. As discussed in the RRTX paper, inevitable collisions can occur if obstacles move too quickly or change their trajectories unpredictably. Parameters like **obstacle inflation** are crucial for safety, but they introduce their own trade-offs. A larger inflation margin creates a safer path but may prevent the robot from finding solutions in narrow passages, such as escaping a corridor between two obstacles. These challenges are fundamental to planning in dynamic environments and can be observed in simulations.

---

## Demos & Visualizations

Here are some visualizations of the planners in action. These demonstrations feature challenging scenarios where obstacles move at high speeds (20-30 m/s), testing the robot's ability to react. The robot's maximum velocity is set to 15 m/s for the R2T and Thruster models, and 10 m/s for the Dubins model. For higher-quality versions of these demos, please see the **[video folder here](https://drive.google.com/drive/folders/1jeUTSjUJhIgvSht9vsUdU3D_7drIBkHU?dmr=1&ec=wgc-drive-globalnav-goto)**.


> **Please Note**: For the kinodynamic planners (Dubins and Thruster), the visualized graph edges are simplified as straight lines and do not represent the true, curved trajectories between nodes. The robot visualization is also not to scale.

### FMTX Planner

| Scenario & Command | Visualization |
| :--- | :--- |
| **Geometric** <br> `./test_fmtx --samples 10000` <br> `--factor 1.5 --seed 42 --duration 30` <br> (Zero inflation) | <p align="center"><img src="./Geometric_S10000_C1_5_I0.gif" alt="Geometric Demo" width="80%"></p> |
| **Kinodynamic (R2T)** <br> `./test_kinodynamic_fmtx_R2T` <br> `--samples 5000 --factor 2.5 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./R2T_S5000_C2_5_I0_5.gif" alt="R2T Demo" width="80%"></p> |
| **Kinodynamic (Dubins)** <br> `./test_kinodynamic_fmtx_dubin_4D` <br> `--samples 3000 --factor 2.5 --seed 42` <br> (2.0m inflation) | <p align="center"><img src="./Dubins_S3000_C2_5_I2_0.gif" alt="Dubins Demo" width="80%"></p> |
| **Kinodynamic (Thruster)** <br> `./test_kinodynamic_fmtx_thruster_5D` <br> `--samples 1000 --factor 2.5 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./Thruster_S1000_C2_5_I0_5.gif" alt="Thruster Demo 1" width="80%"></p> |
| **Kinodynamic (Thruster)** <br> `./test_kinodynamic_fmtx_thruster_5D` <br> `--samples 2000 --factor 2.5 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./Thruster_S2000_C2_5_I0_5.gif" alt="Thruster Demo 2" width="80%"></p> |
| **Kinodynamic (Thruster)** <br> `./test_kinodynamic_fmtx_thruster_5D` <br> `--samples 1000 --factor 3.0 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./Thruster_S1000_C3_0_I0_5.gif" alt="Thruster Demo 3" width="80%"></p> |

### RRTX Planner

| Scenario & Command | Visualization |
| :--- | :--- |
| **Kinodynamic (R2T)** <br> `./test_kinodynamic_rrtx_R2T` <br> `--samples 3000 --factor 2.0 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./RRTX_R2T_S3000_C2_0_I0_5.gif" alt="RRTX R2T Demo" width="80%"></p> |
| **Kinodynamic (Dubins)** <br> `./test_kinodynamic_rrtx_dubin_4D` <br> `--samples 2000 --factor 2.5 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./RRTX_dubins_S2000_C2_5_I0_5.gif" alt="RRTX Dubins Demo" width="80%"></p> |
| **Kinodynamic (Thruster)** <br> `./test_kinodynamic_rrtx_thruster_5D` <br> `--samples 2000 --factor 2.0 --seed 42` <br> (0.5m inflation) | <p align="center"><img src="./RRTX_thruster_S2000_C2_0_I0_5.gif" alt="RRTX Thruster Demo" width="80%"></p> |

### Anytime Planner Comparison

The following demos show a 10-second benchmark comparison between `BIT*` and my experimental anytime planners.

| Planner | Visualization |
| :--- | :--- |
| **Anytime FMT\*** | <p align="center"><img src="./anyfmt_B25_S10_C100_973.gif" alt="Anytime FMT* Demo" width="80%"></p> |
| **BIT\*** | <p align="center"><img src="./bitstar_B25_S10_C119_776.gif" alt="BIT* Demo" width="80%"></p> |
| **Informed Anytime FMTA\*** | <p align="center"><img src="./informedanyfmta_B25_S10_C118_629.gif" alt="Informed Anytime FMTA* Demo" width="80%"></p> |