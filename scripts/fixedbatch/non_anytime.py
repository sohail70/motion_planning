import os
import subprocess
import yaml

# =====================================================================
# 1. EXPERIMENT CONFIGURATION
# =====================================================================

NUM_SEEDS = 100
START_SEED = 42

ENABLE_AUTO_SHUTDOWN = False
SHUTDOWN_COMMAND = ["sudo", "-n", "/usr/bin/systemctl", "poweroff"]

EXECUTABLE_PATH = "../../../build/main"
SDF_BASE_DIR = "../../../sims"

SDF_FILE_R2       = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_R2T      = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_DUBINS   = "dynamic_world_straight_box_circle_20_dubins.sdf"
SDF_FILE_THRUSTER = "dynamic_world_straight_box_circle_20_thruster.sdf"

ALGORITHMS = ["KinodynamicFMTX", "KinodynamicPRMStarDStarLite"]
# ALGORITHMS = ["KinodynamicFMTX"]

# Two sample budgets to sweep
SAMPLE_COUNTS = [3000, 1000]
# SAMPLE_COUNTS = [1000]

STATE_SPACES = {
    "R2": {
        "state_space_type": "RDT",
        "manager_type": "R2T",
        "duration_limit": 1.0,
        "start_state": [48.0, 48.0],
        "goal_state": [-48.0, -48.0],
        "dimensions": 2,
        "bounds_min": [-50.0, -50.0],
        "bounds_max": [50.0, 50.0],
        "min_velocity": 0.0,
        "max_velocity": 20.0,
        "robot_velocity": 10.0,
        "num_of_samples": 3000,
        "factor": 3.0,
        "kd_dim": 2,
        "is_geometric_mode": True,
        "partial_update": False,
        "sdf_file": SDF_FILE_R2,
        "inflation": 0.0
    },
    "R2T": {
        "state_space_type": "RDT",
        "manager_type": "R2T",
        "time_budget": 25.0,
        "start_state": [48.0, 48.0],
        "goal_state": [-48.0, -48.0],
        "dimensions": 3,
        "bounds_min": [-50.0, -50.0],
        "bounds_max": [50.0, 50.0],
        "min_velocity": 0.0,
        "max_velocity": 20.0,
        "num_of_samples": 3000,
        "factor": 3.0,
        "kd_dim": 3,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        "sdf_file": SDF_FILE_R2T,
        "inflation": 1.0
    },
    "Dubins": {
        "state_space_type": "Dubins",
        "manager_type": "Dubins",
        "time_budget": 30.0,
        "start_state": [48.0, 48.0, -0.785],
        "goal_state": [-48.0, -48.0, -2.356],
        "dimensions": 4,
        "bounds_min": [-50.0, -50.0, -3.14159],
        "bounds_max": [50.0, 50.0, 3.14159],
        "min_velocity": 2.0,
        "max_velocity": 15.0,
        "min_turning_radius": 2.0,
        "num_of_samples": 3000,
        "factor": 3.0,
        "kd_dim": 4,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        "sdf_file": SDF_FILE_DUBINS,
        "inflation": 1.0
    },
    "Thruster": {
        "state_space_type": "Thruster",
        "manager_type": "Thruster",
        "time_budget": 35.0,
        "start_state": [48.0, 48.0, 0.0, 0.0],
        "goal_state": [-48.0, -48.0, 0.0, 0.0],
        "dimensions": 5,
        "bounds_min": [-50.0, -50.0, -10.0, -10.0],
        "bounds_max": [50.0, 50.0, 10.0, 10.0],
        "min_velocity": 0.0,
        "max_velocity": 10.0,
        "max_acceleration": 5.0,
        "num_of_samples": 3000,
        "factor": 3.0,
        "kd_dim": 5,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        "sdf_file": SDF_FILE_THRUSTER,
        "inflation": 1.0
    }
}

# =====================================================================
# 2. YAML GENERATOR
# =====================================================================
def create_yaml(algo_name, space_name, seed, params, filename, output_dir):
    exp_name = f"{algo_name}_{space_name}_seed_{seed}"
    # Use absolute path so the binary can find the SDF regardless of cwd
    full_sdf_path = os.path.abspath(os.path.join(SDF_BASE_DIR, params["sdf_file"]))

    config = {
        "experiment": {
            "name": exp_name,
            "planner_type": algo_name,
            "state_space_type": params["state_space_type"],
            "manager_type": params["manager_type"]
        },
        "simulation": {
            "slice_time": 0.02,
            "seed": int(seed)
        },
        "robot": {
            "start_state": params["start_state"],
            "goal_state": params["goal_state"],
            "dimensions": int(params["dimensions"]),
            "bounds_min": params["bounds_min"],
            "bounds_max": params["bounds_max"],
            "min_velocity": params["min_velocity"],
            "max_velocity": params["max_velocity"],
        },
        "planner_params": {
            "num_of_samples": int(params["num_of_samples"]),
            "factor": float(params["factor"]),
            "kdtree_type": "NanoFlann",
            "partial_update": bool(params["partial_update"]),
            "kd_dim": int(params["kd_dim"]),
            "is_geometric_mode": bool(params["is_geometric_mode"]),
        },
        "manager_params": {
            "vis_frequency_hz": 0
        },
        "gazebo_params": {
            "sdf_path": full_sdf_path,
            "inflation": float(params["inflation"])
        }
    }

    if "num_pillar_nodes" in params:
        config["planner_params"]["num_pillar_nodes"] = int(params["num_pillar_nodes"])
    if "goal_radius" in params:
        config["planner_params"]["goal_radius"] = float(params["goal_radius"])
    if "duration_limit" in params:
        config["simulation"]["duration_limit"] = int(params["duration_limit"])
    if "time_budget" in params:
        config["simulation"]["time_budget"] = float(params["time_budget"])
    if "robot_velocity" in params:
        config["robot"]["robot_velocity"] = float(params["robot_velocity"])
    if "min_turning_radius" in params:
        config["robot"]["min_turning_radius"] = float(params["min_turning_radius"])
    if "max_acceleration" in params:
        config["robot"]["max_acceleration"] = float(params["max_acceleration"])

    if algo_name == "KinodynamicPRMStarDStarLite" and space_name in ("R2T", "Dubins", "Thruster"):
        config["planner_params"]["heuristic"] = True

    with open(filename, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

# =====================================================================
# 3. EXECUTION LOOP
# =====================================================================
def run_experiments():
    script_dir = os.path.abspath(os.path.dirname(__file__))
    executable = os.path.abspath(EXECUTABLE_PATH)
    temp_yaml  = os.path.join(script_dir, "temp_non_anytime_config.yaml")

    total_runs = len(SAMPLE_COUNTS) * len(STATE_SPACES) * len(ALGORITHMS) * NUM_SEEDS
    current_run = 1

    for num_samples in SAMPLE_COUNTS:
        # e.g.  results/m1000/  or  results/m3000/
        output_dir = os.path.join(script_dir, "results", f"m{num_samples}")
        os.makedirs(output_dir, exist_ok=True)

        for space_name, params in STATE_SPACES.items():
            # Override the sample count for this sweep iteration
            run_params = {**params, "num_of_samples": num_samples}

            for algo in ALGORITHMS:
                for seed in range(START_SEED, START_SEED + NUM_SEEDS):
                    print(f"\n=======================================================")
                    print(f" RUN {current_run}/{total_runs}: {algo} | {space_name} | "
                          f"samples={num_samples} | seed={seed}")
                    print(f"=======================================================\n")

                    create_yaml(algo, space_name, seed, run_params, temp_yaml, output_dir)

                    try:
                        # cwd=output_dir → binary writes CSVs into results/m<N>/
                        subprocess.run(
                            [executable, temp_yaml],
                            check=True,
                            cwd=output_dir
                        )
                    except subprocess.CalledProcessError as e:
                        print(f"[ERROR] Crashed on run {current_run}: {e}")
                    except KeyboardInterrupt:
                        print("\n[INFO] Aborted by user.")
                        if os.path.exists(temp_yaml):
                            os.remove(temp_yaml)
                        return

                    current_run += 1

    if os.path.exists(temp_yaml):
        os.remove(temp_yaml)
    print("\n[SUCCESS] All experiment batches finished!")

    if ENABLE_AUTO_SHUTDOWN:
        print("[INFO] Shutting down...")
        subprocess.run(SHUTDOWN_COMMAND, check=True)


if __name__ == "__main__":
    run_experiments()

