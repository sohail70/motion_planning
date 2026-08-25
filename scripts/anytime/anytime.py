
import os
import subprocess
import yaml  # Make sure to run: pip install pyyaml

# =====================================================================
# 1. EXPERIMENT CONFIGURATION
# Modify these variables to change the parameters for all algorithms at once!
# =====================================================================

NUM_SEEDS = 100
START_SEED = 42

# --- NEW: SAMPLES TO TEST ---
# The script will run all algorithms/seeds for 1 sample, then repeat for 3 samples
NUM_SAMPLES_LIST = [3, 1]
# NUM_SAMPLES_LIST = [1]

# --- CONFIGURATION ---
# Set this to True to enable automatic shutdown
ENABLE_AUTO_SHUTDOWN = False
# -n ensures sudo is non-interactive; it fails immediately if a password were required
SHUTDOWN_COMMAND = ["sudo", "-n", "/usr/bin/systemctl", "poweroff"]

# --- PATH CONFIGURATIONS ---
# Convert to ABSOLUTE PATHS so they don't break when we change working directories
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
EXECUTABLE_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../build/main"))
SDF_BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../sims"))

# --- SDF FILE CONFIGURATIONS ---
SDF_FILE_R2       = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_R2T      = "dynamic_world_straight_box_circle_20_r2t.sdf"
# SDF_FILE_R2T      = "static_dynamic_world_straight_box_circle_11.sdf"
SDF_FILE_DUBINS   = "dynamic_world_straight_box_circle_20_dubins.sdf"
# SDF_FILE_DUBINS      = "static_dynamic_world_straight_box_circle_11.sdf"
SDF_FILE_THRUSTER = "dynamic_world_straight_box_circle_20_thruster.sdf"

ALGORITHMS = ["KinodynamicANYFMTX", "KinodynamicANYRRTX", "KinodynamicLLPTStar"]
# ALGORITHMS = ["KinodynamicANYRRTX", "KinodynamicLLPTStar"]
# ALGORITHMS = ["KinodynamicANYFMTX"]

# Define the shared parameters for each State Space
STATE_SPACES = {
    "R2": {
        "state_space_type": "RDT",
        "manager_type": "R2T",
        "duration_limit": 10.0, 
        "start_state": [48.0, 48.0],
        "goal_state": [-48.0, -48.0],
        "dimensions": 2,
        "bounds_min": [-50.0, -50.0],
        "bounds_max": [50.0, 50.0],
        "min_velocity": 0.0,
        "max_velocity": 20.0,
        "robot_velocity": 10.0,
        
        "num_of_samples": 3, # This will be overridden by NUM_SAMPLES_LIST
        "factor": 3.0,
        "delta": 10.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 25.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 30.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 45.0,
        "epsilon": 0.1,
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
# 2. YAML GENERATOR FUNCTION
# =====================================================================
def create_yaml(algo_name, space_name, seed, params, filename):
    
    # We embed the number of samples in the experiment name for clarity
    exp_name = f"{algo_name}_{space_name}_seed_{seed}"
    
    full_sdf_path = os.path.join(SDF_BASE_DIR, params["sdf_file"])
    
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
            "delta": float(params["delta"]),
            "epsilon": float(params["epsilon"]),
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

    if algo_name == "KinodynamicLLPTStar":
        config["planner_params"]["heuristic"] = True

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
    total_runs = len(NUM_SAMPLES_LIST) * len(ALGORITHMS) * len(STATE_SPACES) * NUM_SEEDS
    current_run = 1
    
    # NEW: Loop over the different num_of_samples first
    for num_samples in NUM_SAMPLES_LIST:
        
        # Create a specific folder for this configuration
        output_dir = os.path.abspath(os.path.join(SCRIPT_DIR, f"results_samples_{num_samples}"))
        os.makedirs(output_dir, exist_ok=True)
        
        # Place the temporary yaml inside the specific folder
        temp_yaml_file = os.path.join(output_dir, "temp_batch_config.yaml")
        
        for space_name, params in STATE_SPACES.items():
            
            # Override the dictionary value with the current loop's sample size
            params["num_of_samples"] = num_samples
            
            for algo in ALGORITHMS:
                for seed in range(START_SEED, START_SEED + NUM_SEEDS):
                    print(f"\n=======================================================")
                    print(f" RUN {current_run}/{total_runs}: {algo} | {space_name} | Samples: {num_samples} | Seed: {seed}")
                    print(f"=======================================================\n")
                    
                    # 1. Generate config file
                    create_yaml(algo, space_name, seed, params, temp_yaml_file)
                    
                    # 2. Run the C++ executable
                    command = [EXECUTABLE_PATH, temp_yaml_file]
                    
                    try:
                        # NEW: cwd=output_dir forces the C++ program to run from inside the new folder
                        subprocess.run(command, cwd=output_dir, check=True)
                    except subprocess.CalledProcessError as e:
                        print(f"[ERROR] Simulation crashed on run {current_run}: {e}")
                    except KeyboardInterrupt:
                        print("\n[INFO] Experiment batch aborted by user.")
                        if os.path.exists(temp_yaml_file):
                            os.remove(temp_yaml_file)
                        return
                    
                    current_run += 1
                    
        # Clean up the temporary yaml file for this folder when done
        if os.path.exists(temp_yaml_file):
            os.remove(temp_yaml_file)
            
    print("\n[SUCCESS] All experiment batches finished!")

    if ENABLE_AUTO_SHUTDOWN:
        print("[INFO] Auto-shutdown is enabled. Shutting down the system now...")
        subprocess.run(SHUTDOWN_COMMAND, check=True)

if __name__ == "__main__":
    run_experiments()


'''

import os
import subprocess
import yaml  # Make sure to run: pip install pyyaml

# =====================================================================
# 1. EXPERIMENT CONFIGURATION
# Modify these variables to change the parameters for all algorithms at once!
# =====================================================================

NUM_SEEDS = 100
START_SEED = 42

# --- NEW: SAMPLES TO TEST ---
# The script will run all algorithms/seeds for 1 sample, then repeat for 3 samples
NUM_SAMPLES_LIST = [3, 1]
# NUM_SAMPLES_LIST = [1]

# --- CONFIGURATION ---
# Set this to True to enable automatic shutdown
ENABLE_AUTO_SHUTDOWN = False
# -n ensures sudo is non-interactive; it fails immediately if a password were required
SHUTDOWN_COMMAND = ["sudo", "-n", "/usr/bin/systemctl", "poweroff"]

# --- PATH CONFIGURATIONS ---
# Convert to ABSOLUTE PATHS so they don't break when we change working directories
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
EXECUTABLE_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../build/main"))
SDF_BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../sims"))

# --- SDF FILE CONFIGURATIONS ---
SDF_FILE_R2       = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_R2T      = "dynamic_world_straight_box_circle_20_r2t.sdf"
# SDF_FILE_R2T      = "static_dynamic_world_straight_box_circle_11.sdf"
SDF_FILE_DUBINS   = "dynamic_world_straight_box_circle_20_dubins.sdf"
# SDF_FILE_DUBINS      = "static_dynamic_world_straight_box_circle_11.sdf"
SDF_FILE_THRUSTER = "dynamic_world_straight_box_circle_20_thruster.sdf"

ALGORITHMS = ["KinodynamicANYFMTX", "KinodynamicANYRRTX", "KinodynamicLLPTStar"]
# ALGORITHMS = ["KinodynamicANYRRTX", "KinodynamicLLPTStar"]
# ALGORITHMS = ["KinodynamicANYFMTX"]

# Define the shared parameters for each State Space
STATE_SPACES = {
    "R2": {
        "state_space_type": "RDT",
        "manager_type": "R2T",
        "duration_limit": 10.0, 
        "start_state": [48.0, 48.0],
        "goal_state": [-48.0, -48.0],
        "dimensions": 2,
        "bounds_min": [-50.0, -50.0],
        "bounds_max": [50.0, 50.0],
        "min_velocity": 0.0,
        "max_velocity": 20.0,
        "robot_velocity": 10.0,
        
        "num_of_samples": 3, # This will be overridden by NUM_SAMPLES_LIST
        "factor": 3.0,
        "delta": 10.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 25.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 30.0,
        "epsilon": 0.1,
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
        
        "num_of_samples": 3,
        "factor": 3.0,
        "delta": 45.0,
        "epsilon": 0.1,
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
# 2. YAML GENERATOR FUNCTION
# =====================================================================
def create_yaml(algo_name, space_name, seed, params, filename):
    
    # We embed the number of samples in the experiment name for clarity
    exp_name = f"{algo_name}_{space_name}_seed_{seed}"
    
    full_sdf_path = os.path.join(SDF_BASE_DIR, params["sdf_file"])
    
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
            "delta": float(params["delta"]),
            "epsilon": float(params["epsilon"]),
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

    if algo_name == "KinodynamicLLPTStar":
        config["planner_params"]["heuristic"] = True

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
    total_runs = len(NUM_SAMPLES_LIST) * len(ALGORITHMS) * len(STATE_SPACES) * NUM_SEEDS
    current_run = 1
    
    # NEW: Loop over the different num_of_samples first
    for num_samples in NUM_SAMPLES_LIST:
        
        # Create a specific folder for this configuration
        output_dir = os.path.abspath(os.path.join(SCRIPT_DIR, f"results_samples_{num_samples}"))
        os.makedirs(output_dir, exist_ok=True)
        
        # Place the temporary yaml inside the specific folder
        temp_yaml_file = os.path.join(output_dir, "temp_batch_config.yaml")
        
        for space_name, params in STATE_SPACES.items():
            
            # Override the dictionary value with the current loop's sample size
            params["num_of_samples"] = num_samples
            
            for algo in ALGORITHMS:
                for seed in range(START_SEED, START_SEED + NUM_SEEDS):
                    print(f"\n=======================================================")
                    print(f" RUN {current_run}/{total_runs}: {algo} | {space_name} | Samples: {num_samples} | Seed: {seed}")
                    print(f"=======================================================\n")
                    
                    # 1. Generate config file
                    create_yaml(algo, space_name, seed, params, temp_yaml_file)
                    
                    # 2. Run the C++ executable
                    command = [EXECUTABLE_PATH, temp_yaml_file]
                    
                    try:
                        # NEW: cwd=output_dir forces the C++ program to run from inside the new folder
                        subprocess.run(command, cwd=output_dir, check=True)
                    except subprocess.CalledProcessError as e:
                        print(f"[ERROR] Simulation crashed on run {current_run}: {e}")
                    except KeyboardInterrupt:
                        print("\n[INFO] Experiment batch aborted by user.")
                        if os.path.exists(temp_yaml_file):
                            os.remove(temp_yaml_file)
                        return
                    
                    current_run += 1
                    
        # Clean up the temporary yaml file for this folder when done
        if os.path.exists(temp_yaml_file):
            os.remove(temp_yaml_file)
            
    print("\n[SUCCESS] All experiment batches finished!")

    if ENABLE_AUTO_SHUTDOWN:
        print("[INFO] Auto-shutdown is enabled. Shutting down the system now...")
        subprocess.run(SHUTDOWN_COMMAND, check=True)

if __name__ == "__main__":
    run_experiments()


'''