# Walkthrough: ROS2 Humanoid Arm Pick-and-Place

## What Was Built

Complete ROS2 Jazzy project with **5 packages** and **46 files** for a humanoid arm pick-and-place simulation.

### Package Summary

| Package | Files | Purpose |
|---|---|---|
| `humanoid_description` | 9 | Custom humanoid URDF (torso + head + 6-DOF arm + gripper + RGB-D camera) |
| `humanoid_gazebo` | 5 | Gazebo Harmonic world (table, red cube, target zone) + ros2_control controllers |
| `humanoid_moveit_config` | 8 | MoveIt2 config (SRDF, OMPL, KDL kinematics) + launch files |
| [humanoid_perception](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_perception/resource/humanoid_perception) | 7 | OpenCV object detector (HSV color detection + depth-based 3D localization) |
| [humanoid_bringup](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_bringup/resource/humanoid_bringup) | 7 | Pick-and-place state machine + full demo launch |

### Robot Model

Custom simplified humanoid with:
- **Torso**: White rectangular body with blue accent panel
- **Head**: Sphere with LED-style blue eyes
- **Arm**: 6-DOF (shoulder pitch/roll/yaw → elbow pitch → wrist pitch/roll)
- **Gripper**: 2-finger parallel gripper with mimic joints
- **Camera**: RGB-D sensor on head (Gazebo Harmonic `rgbd_camera`)

### Pipeline Flow
```
IDLE → DETECT → APPROACH → GRASP → LIFT → MOVE → PLACE → OPEN → HOME → IDLE
```

## Key Files

- [humanoid.urdf.xacro](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_description/urdf/humanoid.urdf.xacro) — Main robot assembly
- [pick_place.world](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_gazebo/worlds/pick_place.world) — Gazebo world with table and objects
- [object_detector.py](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_perception/humanoid_perception/object_detector.py) — OpenCV detection node
- [pick_place_node.py](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/src/humanoid_bringup/humanoid_bringup/pick_place_node.py) — State machine node

## Next Steps

1. **Push to GitHub** from this Windows laptop
2. **Clone on Ubuntu 24.04** laptop and follow [setup_guide.md](file:///c:/Users/victo/OneDrive/Bureau/ros2_humanoid/docs/setup_guide.md)
3. **Build & test** — start with `ros2 launch humanoid_description display.launch.py` to validate the URDF
4. **Iterate** — tune joint limits, gripper friction, detection params on the real simulation
