import os
import subprocess
import yaml  # Make sure to run: pip install pyyaml

# =====================================================================
# 1. EXPERIMENT CONFIGURATION
# =====================================================================

NUM_SEEDS = 100
START_SEED = 42

# NUM_SAMPLES_LIST = [3, 1]
NUM_SAMPLES_LIST = [1]

ENABLE_AUTO_SHUTDOWN = True
SHUTDOWN_COMMAND = ["sudo", "-n", "/usr/bin/systemctl", "poweroff"]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
EXECUTABLE_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../build/main"))
SDF_BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../sims"))

SDF_FILE_R2       = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_R2T      = "dynamic_world_straight_box_circle_20_r2t.sdf"
SDF_FILE_DUBINS   = "dynamic_world_straight_box_circle_20_dubins.sdf"
SDF_FILE_THRUSTER = "dynamic_world_straight_box_circle_20_thruster.sdf"

ALGORITHMS = ["KinodynamicANYFMTX", "KinodynamicANYRRTX", "KinodynamicLLPTStar"]

# --- NEW: Added 'time_budgets_list' to each state space ---
STATE_SPACES = {
    # "R2": {
    #     "state_space_type": "RDT",
    #     "manager_type": "R2T",
    #     "time_budgets_list": [10.0, 15.0], # R2 uses duration_limit originally, this replaces it
    #     "duration_limit": 10.0, 
    #     "start_state": [48.0, 48.0],
    #     "goal_state": [-48.0, -48.0],
    #     "dimensions": 2,
    #     "bounds_min": [-50.0, -50.0],
    #     "bounds_max": [50.0, 50.0],
    #     "min_velocity": 0.0,
    #     "max_velocity": 20.0,
    #     "robot_velocity": 10.0,
    #     "num_of_samples": 3, 
    #     "factor": 3.0,
    #     "delta": 10.0,
    #     "epsilon": 0.1,
    #     "kd_dim": 2,
    #     "is_geometric_mode": True,
    #     "partial_update": False,
    #     "sdf_file": SDF_FILE_R2,
    #     "inflation": 0.0
    # },
    
    # "R2T": {
    #     "state_space_type": "RDT",
    #     "manager_type": "R2T",
    #     "time_budgets_list": [20.0, 25.0, 30.0], # Multiple time budgets for R2T
    #     "time_budget": 25.0,
    #     "start_state": [48.0, 48.0],
    #     "goal_state": [-48.0, -48.0],
    #     "dimensions": 3,
    #     "bounds_min": [-50.0, -50.0],
    #     "bounds_max": [50.0, 50.0],
    #     "min_velocity": 0.0,
    #     "max_velocity": 20.0,
    #     "num_of_samples": 3,
    #     "factor": 3.0,
    #     "delta": 25.0,
    #     "epsilon": 0.1,
    #     "kd_dim": 3,
    #     "is_geometric_mode": False,
    #     "partial_update": True,
    #     "num_pillar_nodes": 0,
    #     "goal_radius": 0.5,
    #     "sdf_file": SDF_FILE_R2T,
    #     "inflation": 1.0
    # },
    
    # "Dubins": {
    #     "state_space_type": "Dubins",
    #     "manager_type": "Dubins",
    #     "time_budgets_list": [30.0, 45.0],
    #     "time_budget": 30.0,
    #     "start_state": [48.0, 48.0, -0.785],
    #     "goal_state": [-48.0, -48.0, -2.356],
    #     "dimensions": 4,
    #     "bounds_min": [-50.0, -50.0, -3.14159],
    #     "bounds_max": [50.0, 50.0, 3.14159],
    #     "min_velocity": 2.0,
    #     "max_velocity": 15.0,
    #     "min_turning_radius": 2.0,
    #     "num_of_samples": 3,
    #     "factor": 3.0,
    #     "delta": 30.0,
    #     "epsilon": 0.1,
    #     "kd_dim": 4,
    #     "is_geometric_mode": False,
    #     "partial_update": True,
    #     "num_pillar_nodes": 0,
    #     "goal_radius": 0.5,
    #     "sdf_file": SDF_FILE_DUBINS,
    #     "inflation": 1.0
    # },
    
    "Thruster": {
        "state_space_type": "Thruster",
        "manager_type": "Thruster",
        "time_budgets_list": [65.0 , 55.0 ,45.0, 35.0],
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
        config["simulation"]["duration_limit"] = float(params["duration_limit"]) 
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
    # Dynamically calculate the total number of runs based on the different time budget lists
    total_runs = 0
    for num_samples in NUM_SAMPLES_LIST:
        for space_name, params in STATE_SPACES.items():
            t_list = params.get("time_budgets_list", [0])
            total_runs += len(t_list) * len(ALGORITHMS) * NUM_SEEDS
            
    current_run = 1
    
    for num_samples in NUM_SAMPLES_LIST:
        for space_name, params in STATE_SPACES.items():
            # Get the list of time budgets for this specific state space
            time_budgets = params.get("time_budgets_list", [params.get("time_budget", params.get("duration_limit", 10.0))])
            
            for t_budget in time_budgets:
                # 1. Update the time budget and sample parameters dynamically for the YAML generator
                params["num_of_samples"] = num_samples
                if "duration_limit" in params:
                    params["duration_limit"] = t_budget
                if "time_budget" in params:
                    params["time_budget"] = t_budget
                
                # 2. Create a unique folder utilizing State Space, Sample Count, and Time Budget
                dir_name = f"results_{space_name}_samples_{num_samples}_time_{t_budget}"
                output_dir = os.path.abspath(os.path.join(SCRIPT_DIR, dir_name))
                os.makedirs(output_dir, exist_ok=True)
                
                temp_yaml_file = os.path.join(output_dir, "temp_batch_config.yaml")
                
                for algo in ALGORITHMS:
                    for seed in range(START_SEED, START_SEED + NUM_SEEDS):
                        print(f"\n=======================================================")
                        print(f" RUN {current_run}/{total_runs}: {algo} | {space_name} | Samples: {num_samples} | Time: {t_budget} | Seed: {seed}")
                        print(f"=======================================================\n")
                        
                        create_yaml(algo, space_name, seed, params, temp_yaml_file)
                        
                        command = [EXECUTABLE_PATH, temp_yaml_file]
                        
                        try:
                            # C++ program runs from inside the unique new folder
                            subprocess.run(command, cwd=output_dir, check=True)
                        except subprocess.CalledProcessError as e:
                            print(f"[ERROR] Simulation crashed on run {current_run}: {e}")
                        except KeyboardInterrupt:
                            print("\n[INFO] Experiment batch aborted by user.")
                            if os.path.exists(temp_yaml_file):
                                os.remove(temp_yaml_file)
                            return
                        
                        current_run += 1
                        
                # Clean up the temporary yaml file for this specific folder when done
                if os.path.exists(temp_yaml_file):
                    os.remove(temp_yaml_file)
            
    print("\n[SUCCESS] All experiment batches finished!")

    if ENABLE_AUTO_SHUTDOWN:
        print("[INFO] Auto-shutdown is enabled. Shutting down the system now...")
        subprocess.run(SHUTDOWN_COMMAND, check=True)

if __name__ == "__main__":
    run_experiments()
