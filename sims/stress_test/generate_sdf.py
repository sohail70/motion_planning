#!/usr/bin/env python3
"""
Generate a random SDF file with dynamic obstacles for stress‑testing planners.
Obstacles are placed so that they repeatedly cross the straight line from start to goal.
"""

import argparse
import math
import random

# Fixed workspace geometry (metres)
START = (48.0, 48.0)
GOAL  = (-48.0, -48.0)

# Obstacle appearance (defaults, can be overridden via command line)
DEFAULT_OBSTACLE_RADIUS = 3.0
OBSTACLE_LENGTH = 1.0          # cylinder height (visual only)

# Motion limits
MOTION_LIMIT_MIN = 20.0        # metres before turning around
MOTION_LIMIT_MAX = 40.0

# Bias: obstacles cross the start-goal line
BIAS_PERPENDICULAR = False
PERP_ANGLE_TOLERANCE = 30.0    # degrees around the true perpendicular

# Synchronised mode parameters (used when --sync is given)
SYNC_OFFSET   = 10.0          # fixed perpendicular distance from the start–goal line
SYNC_MARGIN   = 5.0           # fixed extra distance after crossing the line
# The total amplitude becomes SYNC_OFFSET + SYNC_MARGIN = 15.0 m

# SDF template strings
SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <surface>
            <contact>
              <collide_without_contact_bitmask>0x01</collide_without_contact_bitmask>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_FOOTER = """
    <include>
      <uri>/home/sohail/gazeb/GAZEBO_MOV/model.sdf</uri>
      <name>tugbot</name>
      <pose>48.0 48.0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
"""

OBSTACLE_TEMPLATE = """
<model name="{name}">
  <pose>{px} {py} 0 0 0 0</pose>
  <link name="moving_link">
    <gravity>false</gravity>
    <visual name="visual">
      <geometry>
        <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
      </geometry>
      <material><ambient>1 0 0 1</ambient></material>
    </visual>
  </link>
  <plugin name="MoverPluginC" filename="/home/sohail/gazeb/GAZEBO_MOV/build/libMoverPluginC.so">
    <link_name>moving_link</link_name>
    <direction>{dx} {dy} 0</direction>
    <amplitude>{amplitude}</amplitude>
    <speed>{speed}</speed>
    <turnaround_threshold>0.5</turnaround_threshold>
  </plugin>
</model>
"""

def generate_obstacle(index, speed, obstacle_radius, bias_perpendicular, sync_enabled, random_placement=False, sync_amplitude=None):
    """
    Returns a dictionary with the parameters needed to fill OBSTACLE_TEMPLATE.
    The obstacle is placed either randomly in the workspace or relative to the start–goal line.
    """
    # ------------------------------------------------------------
    # RANDOM PLACEMENT MODE – no relation to start–goal line
    # ------------------------------------------------------------
    if random_placement:
        while True:
            # candidate position
            px = random.uniform(-48, 48)
            py = random.uniform(-48, 48)
            # must be clear of start/goal
            if ((px-START[0])**2 + (py-START[1])**2) < 25 or \
               ((px-GOAL[0])**2  + (py-GOAL[1])**2)  < 25:
                continue

            # random direction
            angle = random.uniform(0, 2*math.pi)
            dx = math.cos(angle)
            dy = math.sin(angle)

            # random amplitude (e.g., 20–80 m)
            amplitude = random.uniform(20, 80)

            # extreme points of the oscillation
            end1_x = px + amplitude * dx
            end1_y = py + amplitude * dy
            end2_x = px - amplitude * dx
            end2_y = py - amplitude * dy

            # check if both ends stay within [-50,50] (with 0.5 m margin)
            margin = 0.5
            if (end1_x > 50-margin or end1_x < -50+margin or
                end1_y > 50-margin or end1_y < -50+margin or
                end2_x > 50-margin or end2_x < -50+margin or
                end2_y > 50-margin or end2_y < -50+margin):
                continue   # try another placement

            break   # valid placement found

        return {
            'name': f'moving_cylinder_{index}',
            'px': px,
            'py': py,
            'radius': obstacle_radius,
            'length': OBSTACLE_LENGTH,
            'dx': dx,
            'dy': dy,
            'amplitude': amplitude,
            'speed': speed,
        }

    # ------------------------------------------------------------
    # Original behaviour (start–goal line reference)
    # ------------------------------------------------------------
    # 1. Pick a random crossing point on the start‑goal line
    t = random.uniform(0.1, 0.9)
    cx = START[0] + t * (GOAL[0] - START[0])
    cy = START[1] + t * (GOAL[1] - START[1])

    # 2. Compute the true perpendicular direction to that line
    dx_line = GOAL[0] - START[0]
    dy_line = GOAL[1] - START[1]
    true_perp_angle = math.atan2(dy_line, dx_line) + math.pi / 2.0

    # Randomly flip to the other side
    if random.random() < 0.5:
        true_perp_angle += math.pi

    # 3. Apply a bias: angle = true_perp_angle ± tolerance°
    if bias_perpendicular:
        angle = true_perp_angle + math.radians(
            random.uniform(-PERP_ANGLE_TOLERANCE, PERP_ANGLE_TOLERANCE)
        )
    else:
        angle = random.uniform(0, 2 * math.pi)

    # ---- SYNCHRONISED MODE ----
    if sync_enabled:
        offset = SYNC_OFFSET * random.choice([-1, 1])
        angle = true_perp_angle
        amplitude = sync_amplitude if sync_amplitude is not None else (SYNC_OFFSET + SYNC_MARGIN)
    else:
        offset = random.uniform(2.0, 20.0) * random.choice([-1, 1])
        margin = random.uniform(5.0, 15.0)
        amplitude = abs(offset) + margin
        amplitude = max(amplitude, MOTION_LIMIT_MIN)
        amplitude = min(amplitude, MOTION_LIMIT_MAX)

    init_x = cx + offset * math.cos(true_perp_angle)
    init_y = cy + offset * math.sin(true_perp_angle)

    dir_x = math.cos(angle)
    dir_y = math.sin(angle)

    return {
        'name': f'moving_cylinder_{index}',
        'px': init_x,
        'py': init_y,
        'radius': obstacle_radius,
        'length': OBSTACLE_LENGTH,
        'dx': dir_x,
        'dy': dir_y,
        'amplitude': amplitude,
        'speed': speed,
    }


def generate_sdf(obstacles, output_file):
    """Write the SDF file to output_file."""
    with open(output_file, 'w') as f:
        f.write(SDF_HEADER)
        for obs in obstacles:
            f.write(OBSTACLE_TEMPLATE.format(**obs))
        f.write(SDF_FOOTER)


def main():
    parser = argparse.ArgumentParser(
        description='Generate a random SDF file with dynamic obstacles.'
    )
    parser.add_argument('--speed', type=float, required=True,
                        help='Obstacle speed (m/s)')
    parser.add_argument('--nobs', type=int, default=20,
                        help='Number of dynamic obstacles')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility')
    parser.add_argument('--out', type=str, default='random_obstacles.sdf',
                        help='Output SDF file name')
    parser.add_argument('--radius', type=float, default=DEFAULT_OBSTACLE_RADIUS,
                        help=f'Obstacle cylinder radius (default {DEFAULT_OBSTACLE_RADIUS})')
    parser.add_argument('--no-bias', action='store_true',
                        help='Use uniformly random motion axes (no perpendicular bias)')

    parser.add_argument('--sync', action='store_true',
                        help='Synchronise all obstacles (identical amplitude, offset, and perpendicular axes)')

    parser.add_argument('--random-placement', action='store_true',
                        help='Place obstacles uniformly at random (ignore start-goal line)')

    parser.add_argument('--sync-amplitude', type=float, default=None,
                        help='Custom amplitude for sync mode (overrides SYNC_OFFSET+SYNC_MARGIN)')



    args = parser.parse_args()
    sync_enabled = args.sync

    if args.seed is not None:
        random.seed(args.seed)

    # Whether to bias motion axes perpendicular to start-goal line
    bias = not args.no_bias
    radius = args.radius

    random_placement = args.random_placement

    obstacles = [generate_obstacle(i, args.speed, radius, bias, sync_enabled, random_placement, args.sync_amplitude)
                 for i in range(args.nobs)]

    generate_sdf(obstacles, args.out)
    print(f"Generated {args.nobs} obstacles with speed {args.speed} m/s -> {args.out}")


if __name__ == '__main__':
    main()
