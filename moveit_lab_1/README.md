# MoveIt Lab 1 - Obstacle Avoidance Demo

This package demonstrates basic motion planning and obstacle avoidance using MoveIt 2 in ROS 2 Humble.

## Overview

The demo showcases:
- Planning and executing robot motion to target poses
- Interactive visualization in RViz2
- Adding collision objects to the planning scene
- Planning trajectories that avoid obstacles

## Robot Configuration

- **Planning Group**: `arm` (5 DOF)
- **End Effector**: `gripper`
- **Robot**: so101_new_calib

## Building

```bash

colcon build --packages-select moveit_lab_1
source install/setup.bash
```

## Running Modes

### 🎬 Mode 1: Demo Mode (Default)
Runs the automated demonstration with interactive prompts:

```bash
ros2 launch moveit_lab_1 hello_moveit.launch.py
```

**Demo Steps:**
1. RViz opens with the robot visualization
2. Click **"Next"** to plan the first movement
3. Click **"Next"** to execute the plan
4. A **green box obstacle** appears in the scene
5. Click **"Next"** to plan around the obstacle
6. Click **"Next"** to execute obstacle avoidance
7. **Demo completes** → You can now use RViz manually!

After the demo, the obstacle stays in the scene and you can use the RViz Motion Planning panel freely.

### 🎮 Mode 2: Manual Mode Only
Skips the demo and goes straight to manual control:

```bash
ros2 launch moveit_lab_1 hello_moveit.launch.py demo:=false
```

The obstacle is present, and you can immediately use RViz interactive markers.

## Using RViz Interactive Markers

After demo completes (or in manual mode):

1. **Select Goal:** In the Motion Planning panel → "Planning" tab
   - Option A: Use **Joints** tab and move sliders
   - Option B: Select **`<random valid>`** from Goal State dropdown
   
2. **Plan:** Click **"Plan"** to see the trajectory (green = valid)

3. **Execute:** Click **"Execute"** to run the planned motion

4. **Or use "Plan & Execute"** to do both steps at once

### 💡 Tips for Interactive Control:

- **Green marker** = Reachable position ✅
- **Blue marker** = Invalid/unreachable ❌
- Use **Joints tab** for guaranteed valid positions
- **`<random valid>`** generates random reachable poses
- Click **"Update"** on Start State if needed

## What You'll See

- **Initial pose**: Robot moves to `(x=0.28, y=-0.2, z=0.5)`
- **Obstacle**: Green box at `(x=0.2, y=0.0, z=0.25)`, size 0.5 x 0.1 x 0.5 m
- **Obstacle avoidance**: Robot moves to `(x=0.28, y=0.2, z=0.5)` while avoiding the box

## Files

- [`src/hello_moveit.cpp`](src/hello_moveit.cpp) - Main demo node
- [`launch/hello_moveit.launch.py`](launch/hello_moveit.launch.py) - Launch file

## Dependencies

- moveit_ros_planning_interface
- moveit_visual_tools
- rclcpp
- geometry_msgs

## Learning Objectives

✅ Understanding `MoveGroupInterface` for motion planning  
✅ Using `PlanningSceneInterface` to add collision objects  
✅ Visualizing plans with `MoveItVisualTools`  
✅ Interactive step-by-step execution in RViz2  
✅ Manual control via RViz Motion Planning panel
