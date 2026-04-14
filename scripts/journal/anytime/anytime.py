import os
import subprocess
import yaml  # Make sure to run: pip install pyyaml

# =====================================================================
# 1. EXPERIMENT CONFIGURATION
# Modify these variables to change the parameters for all algorithms at once!
# =====================================================================

NUM_SEEDS = 5
START_SEED = 42

# --- PATH CONFIGURATIONS ---
# Define where your C++ executable is located
EXECUTABLE_PATH = "../../../build/main"

# Define the folder where your Gazebo SDF files are located
SDF_BASE_DIR = "../../../sims"

# --- SDF FILE CONFIGURATIONS (Change these easily here!) ---
SDF_FILE_R2       = "dynamic_world_straight_box_circle_10.sdf"
SDF_FILE_R2T      = "dynamic_world_straight_box_circle_10.sdf"
SDF_FILE_DUBINS   = "dynamic_world_straight_box_circle_10.sdf"
SDF_FILE_THRUSTER = "dynamic_world_straight_box_circle_10_slow.sdf"


ALGORITHMS = ["KinodynamicANYFMTX", "KinodynamicANYRRTX"]

# Define the shared parameters for each State Space
STATE_SPACES = {
    "R2": {
        "state_space_type": "RDT",
        "manager_type": "R2T",
        "duration_limit": 10.0, # R2 uses duration_limit instead of time_budget
        "start_state": [48.0, 48.0],
        "goal_state": [-48.0, -48.0],
        "dimensions": 2,
        "bounds_min": [-50.0, -50.0],
        "bounds_max": [50.0, 50.0],
        "min_velocity": 0.0,
        "max_velocity": 20.0,
        "robot_velocity": 10.0,
        
        "num_of_samples": 1,
        "factor": 2.0,
        "delta": 10.0,
        "epsilon": 0.1,
        "kd_dim": 2,
        "is_geometric_mode": True,
        "partial_update": False,
        
        # Pulls the variable defined at the top!
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
        
        "num_of_samples": 1,
        "factor": 2.0,
        "delta": 20.0,
        "epsilon": 0.1,
        "kd_dim": 3,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        
        "sdf_file": SDF_FILE_R2T,
        "inflation": 0.75
    },
    
    "Dubins": {
        "state_space_type": "Dubins",
        "manager_type": "Dubins",
        "time_budget": 25.0,
        "start_state": [48.0, 48.0, -0.785],
        "goal_state": [-48.0, -48.0, -2.356],
        "dimensions": 4,
        "bounds_min": [-50.0, -50.0, -3.14159],
        "bounds_max": [50.0, 50.0, 3.14159],
        "min_velocity": 2.0,
        "max_velocity": 20.0,
        "min_turning_radius": 2.0,
        
        "num_of_samples": 1,
        "factor": 3.0,
        "delta": 20.0,
        "epsilon": 0.1,
        "kd_dim": 4,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        
        "sdf_file": SDF_FILE_DUBINS,
        "inflation": 0.75
    },
    
    "Thruster": {
        "state_space_type": "Thruster",
        "manager_type": "Thruster",
        "time_budget": 25.0,
        "start_state": [48.0, 48.0, 0.0, 0.0],
        "goal_state": [-48.0, -48.0, 0.0, 0.0],
        "dimensions": 5,
        "bounds_min": [-50.0, -50.0, -15.0, -15.0],
        "bounds_max": [50.0, 50.0, 15.0, 15.0],
        "min_velocity": 0.0,
        "max_velocity": 15.0,
        "max_acceleration": 5.0,
        
        "num_of_samples": 1,
        "factor": 3.0,
        "delta": 40.0,
        "epsilon": 0.1,
        "kd_dim": 5,
        "is_geometric_mode": False,
        "partial_update": True,
        "num_pillar_nodes": 0,
        "goal_radius": 0.5,
        
        "sdf_file": SDF_FILE_THRUSTER,
        "inflation": 0.75
    }
}

# =====================================================================
# 2. YAML GENERATOR FUNCTION
# =====================================================================
def create_yaml(algo_name, space_name, seed, params, filename):
    
    exp_name = f"{algo_name}_{space_name}_seed_{seed}"
    
    # Automatically join the base directory with the file name
    full_sdf_path = os.path.join(SDF_BASE_DIR, params["sdf_file"])
    
    config = {
        "experiment": {
            "name": exp_name,
            "planner_type": algo_name,
            "state_space_type": params["state_space_type"],
            "manager_type": params["manager_type"]
        },
        "simulation": {
            "slice_time": 0.02 if space_name != "R2" else 0.05,
            "seed": int(seed) # FORCE INT
        },
        "robot": {
            "start_state": params["start_state"],
            "goal_state": params["goal_state"],
            "dimensions": int(params["dimensions"]), # FORCE INT
            "bounds_min": params["bounds_min"],
            "bounds_max": params["bounds_max"],
            "min_velocity": params["min_velocity"],
            "max_velocity": params["max_velocity"],
        },
        "planner_params": {
            "num_of_samples": int(params["num_of_samples"]), # FORCE INT
            "factor": float(params["factor"]),
            "delta": float(params["delta"]),
            "epsilon": float(params["epsilon"]),
            "kdtree_type": "NanoFlann",
            "partial_update": bool(params["partial_update"]),
            "kd_dim": int(params["kd_dim"]), # FORCE INT
            "is_geometric_mode": bool(params["is_geometric_mode"]),
        },
        "manager_params": {
            "vis_frequency_hz": 30
        },
        "gazebo_params": {
            "sdf_path": full_sdf_path,
            "inflation": float(params["inflation"])
        }
    }

    # Handle conditional parameters safely and cast correctly
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

    with open(filename, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, sort_keys=False)

# =====================================================================
# 3. EXECUTION LOOP
# =====================================================================
def run_experiments():
    temp_yaml_file = "temp_batch_config.yaml"
    
    total_runs = len(ALGORITHMS) * len(STATE_SPACES) * NUM_SEEDS
    current_run = 1
    
    for space_name, params in STATE_SPACES.items():
        for algo in ALGORITHMS:
            for seed in range(START_SEED, START_SEED + NUM_SEEDS):
                print(f"\n=======================================================")
                print(f" RUN {current_run}/{total_runs}: {algo} | {space_name} | Seed: {seed}")
                print(f"=======================================================\n")
                
                # 1. Generate the config file for this specific run
                create_yaml(algo, space_name, seed, params, temp_yaml_file)
                
                # 2. Run the C++ executable
                command = [EXECUTABLE_PATH, temp_yaml_file]
                
                try:
                    subprocess.run(command, check=True)
                except subprocess.CalledProcessError as e:
                    print(f"[ERROR] Simulation crashed on run {current_run}: {e}")
                except KeyboardInterrupt:
                    print("\n[INFO] Experiment batch aborted by user.")
                    if os.path.exists(temp_yaml_file):
                        os.remove(temp_yaml_file)
                    return
                
                current_run += 1
                
    # Clean up the temporary yaml file when done
    if os.path.exists(temp_yaml_file):
        os.remove(temp_yaml_file)
    print("\n[SUCCESS] All experiment batches finished!")

if __name__ == "__main__":
    run_experiments()