DUAL G1 ISAAC LAB ENVIRONMENT
SAFE Robotics Lab - Georgia Tech

REQUIREMENTS
- Ubuntu 22.04
- NVIDIA GPU with CUDA
- Isaac Sim 4.5.0 or 5.x
- Isaac Lab 2.3+

SETUP
1. Install Isaac Lab following: https://isaac-sim.github.io/IsaacLab/
2. Clone unitree assets: https://github.com/unitreerobotics/unitree_sim_isaaclab
3. Place these files in your Isaac Lab project directory

FILES
dual_g1_scene_cfg.py    - Scene with two G1 robots, table, lighting
dual_g1_env_cfg.py      - Environment config (obs, actions, events, sim params)
dual_g1_teleop.py       - Teleoperation interface (keyboard + ZMQ)
run_dual_g1.py          - Main launch script
inspire_hand_cfg.py     - Inspire hand config (update USD paths when URDF converted)

USAGE

Standalone (both robots idle, verify scene loads):
    ./isaaclab.sh -p run_dual_g1.py

Keyboard teleoperation:
    ./isaaclab.sh -p run_dual_g1.py --teleop

ZMQ teleoperation (for external teleop system):
    ./isaaclab.sh -p run_dual_g1.py --teleop_zmq --left_port 5555 --right_port 5556

Multiple parallel environments:
    ./isaaclab.sh -p run_dual_g1.py --num_envs 16

INSPIRE HAND INTEGRATION
1. Convert URDF to USD:
    python -c "from inspire_hand_cfg import convert_urdf_to_usd; convert_urdf_to_usd('path/to/inspire_hand.urdf', './converted_assets/')"
2. Update USD paths in inspire_hand_cfg.py
3. Add hand articulations to DualG1SceneCfg and attach to G1 wrist frames

ZMQ TELEOP PROTOCOL
Send 29 float32 values (joint positions in radians) to each port.
Left robot: tcp://localhost:5555
Right robot: tcp://localhost:5556

KEYBOARD CONTROLS
1/2         Switch active robot (left/right)
q/w/e/r/t   Switch joint group (left_arm/right_arm/waist/left_leg/right_leg)
i/k         Joint 0 +/-
j/l         Joint 1 +/-
u/o         Joint 2 +/-
0           Reset all targets
