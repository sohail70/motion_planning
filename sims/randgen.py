import random

def generate_sdfs(min_speed, max_speed, output_full="full_20.world", output_subset="subset_10.world", keep_indices=None):
    """
    Generate two SDF files:
    - full: all 20 obstacles with random speeds between min_speed and max_speed.
    - subset: only obstacles at the given indices (0‑based).
    
    Default keep_indices = list(range(0,5)) + list(range(15,20))  # cylinders 1-5 and boxes 6-10
    """
    if keep_indices is None:
        keep_indices = list(range(0,5)) + list(range(15,20))   # 0‑4 and 15‑19 → total 10

    # --- 1. Fixed parts of the SDF (header & footer) ---------------------------------
    header = '''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
  
<physics type="ode">
    <max_step_size>0.01</max_step_size>
    <real_time_update_rate>60</real_time_update_rate>
</physics>  
  
    <!-- Add a ground plane with a lighter color -->
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
      <size>100 100</size>
        </plane>
      </geometry>
      <!-- Disable collisions with kinematic objects -->
      <surface>
        <contact>
          <collide_without_contact_bitmask>0x01</collide_without_contact_bitmask>
        </contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>
</model>

'''

    footer = '''
<!--****** -->
    <include>
      <uri>/home/sohail/gazeb/GAZEBO_MOV/model.sdf</uri>
      <name>tugbot</name>
      <pose>48.0 48.0 0.1 0 0 0</pose>

<!-- 
  <plugin filename="/home/sohail/gazeb/GAZEBO_MOV/build/libTeleportPlugin.so" name="TeleportPlugin">
    <speed>0.8</speed>
    <interpolation_step>0.3</interpolation_step>
  </plugin> -->

    </include>

  </world>
</sdf>'''

    # --- 2. Definition of the 20 obstacles (name, pose, direction, amplitude, geometry type) -----
    obstacles = [
        # Cylinders 1..5 (left‑right, positive x)
        ("moving_cylinder_1", "-55.0 30.0 0 0 0 0", "1 0 0", "cylinder"),
        ("moving_cylinder_2", "-55.0 15.0 0 0 0 0", "1 0 0", "cylinder"),
        ("moving_cylinder_3", "-55.0 0.0 0 0 0 0", "1 0 0", "cylinder"),
        ("moving_cylinder_4", "-55.0 -15 0 0 0 0", "1 0 0", "cylinder"),
        ("moving_cylinder_5", "-55.0 -30 0 0 0 0", "1 0 0", "cylinder"),
        # Cylinders 6..10 (up‑down, positive y)
        ("moving_cylinder_6", "-30.0 -55.0 0 0 0 0", "0 1 0", "cylinder"),
        ("moving_cylinder_7", "-15.0 -55.0 0 0 0 0", "0 1 0", "cylinder"),
        ("moving_cylinder_8", "0.0 -55.0 0 0 0 0", "0 1 0", "cylinder"),
        ("moving_cylinder_9", "15.0 -55 0 0 0 0", "0 1 0", "cylinder"),
        ("moving_cylinder_10", "30.0 -55 0 0 0 0", "0 1 0", "cylinder"),
        # Boxes 1..5 (left‑right, negative x)
        ("moving_box_1", "55.0 30.0 0 0 0 0", "-1 0 0", "box"),
        ("moving_box_2", "55.0 15.0 0 0 0 0", "-1 0 0", "box"),
        ("moving_box_3", "55.0 0.0 0 0 0 0", "-1 0 0", "box"),
        ("moving_box_4", "55.0 -15 0 0 0 0", "-1 0 0", "box"),
        ("moving_box_5", "55.0 -30 0 0 0 0", "-1 0 0", "box"),
        # Boxes 6..10 (up‑down, negative y)
        ("moving_box_6", "-30.0 55.0 0 0 0 0", "0 -1 0", "box"),
        ("moving_box_7", "-15.0 55.0 0 0 0 0", "0 -1 0", "box"),
        ("moving_box_8", "0.0 55.0 0 0 0 0", "0 -1 0", "box"),
        ("moving_box_9", "15.0 55 0 0 0 0", "0 -1 0", "box"),
        ("moving_box_10", "30.0 55 0 0 0 0", "0 -1 0", "box"),
    ]

    # --- 3. Generate 20 random speeds (same for both outputs) -------------------------
    speeds = [random.uniform(min_speed, max_speed) for _ in range(20)]

    # --- 4. Helper to build one obstacle block ----------------------------------------
    def make_obstacle(name, pose, direction, geom_type, speed):
        if geom_type == "cylinder":
            geometry = '''      <geometry>
        <cylinder>
          <radius>4.0</radius>
          <length>1.0</length>
        </cylinder>
      </geometry>'''
        else:  # box
            geometry = '''      <geometry>
        <box><size>8.0 8.0 1.0</size></box>
      </geometry>'''

        return f'''
<model name="{name}">
<pose>{pose}</pose>
    <link name="moving_link">
                  <gravity>false</gravity>
    <visual name="visual">
{geometry}
      <material>
        <ambient>0 0 1 1</ambient>
      </material>
    </visual>
    </link>

    <plugin name="MoverPluginC" filename="/home/sohail/gazeb/GAZEBO_MOV/build/libMoverPluginC.so">
        <link_name>moving_link</link_name>
        <direction>{direction}</direction>
        <amplitude>110.0</amplitude>
        <speed>{speed:.4f}</speed>
        <turnaround_threshold>0.5</turnaround_threshold>
    </plugin>
</model>'''

    # --- 5. Build full SDF (all 20) ---------------------------------------------------
    full_body = ""
    for (name, pose, direction, geom), sp in zip(obstacles, speeds):
        full_body += make_obstacle(name, pose, direction, geom, sp)

    full_sdf = header + full_body + footer
    with open(output_full, 'w') as f:
        f.write(full_sdf)

    # --- 6. Build subset SDF (only selected indices) ----------------------------------
    subset_body = ""
    for idx in keep_indices:
        if idx < 0 or idx >= len(obstacles):
            continue
        name, pose, direction, geom = obstacles[idx]
        sp = speeds[idx]
        subset_body += make_obstacle(name, pose, direction, geom, sp)

    subset_sdf = header + subset_body + footer
    with open(output_subset, 'w') as f:
        f.write(subset_sdf)

    # --- 7. Report -------------------------------------------------------------------
    print(f"✅ Full 20‑obstacle SDF saved as: {output_full}")
    print(f"✅ Subset 10‑obstacle SDF saved as: {output_subset}")
    print("\nGenerated speeds (full list):")
    for i, obs in enumerate(obstacles, 1):
        name = obs[0]          # first element is the name
        sp = speeds[i-1]
        print(f"{i:2d}. {name:20s} : {sp:.4f}")
    print("\nKept indices for subset:", keep_indices)
    print("Subset is also uniformly random because indices were fixed before generation.")

if __name__ == "__main__":
    min_sp = float(input("Enter minimum speed: "))
    max_sp = float(input("Enter maximum speed: "))
    generate_sdfs(min_sp, max_sp, "full_20.world", "subset_10.world")
