# MTC Tutorial - Pick and Place Demo

This package demonstrates advanced pick-and-place operations using the MoveIt Task Constructor (MTC) in ROS 2 Humble.

## Overview

The demo implements a complete pick-and-place operation with multiple task stages:
- Opening/closing the gripper
- Approaching and grasping an object
- Lifting the object
- Moving to place location
- Placing and releasing the object
- Returning to home position

## Robot Configuration

- **Arm Planning Group**: `arm` (5 DOF)
- **Gripper Planning Group**: `gripper` (1 DOF)
- **End Effector**: `gripper`
- **Robot**: so101_new_calib

## Building

```bash
cd /home/rafae/ws_lab3
colcon build --packages-select mtc_tutorial
source install/setup.bash
```

## Running the Demo

```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```

## Demo Behavior

The node automatically:
1. Creates a **cylindrical object** in the planning scene (height 0.1m, diameter 0.02m)
2. Plans the complete **pick-and-place sequence**
3. Publishes the solution for visualization
4. **Executes** the planned task

## Task Stages

The MTC task consists of these stages:

1. **current** - Capture initial state
2. **open hand** - Open gripper
3. **move to pick** - Approach pick location
4. **pick object** (serial container):
   - approach object
   - generate grasp pose
   - grasp pose IK
   - allow collision (hand,object)
   - close hand
   - attach object
   - lift object
5. **move to place** - Move to place location
6. **place object** (serial container):
   - generate place pose
   - place pose IK
   - open hand
   - forbid collision (hand,object)
   - detach object
   - retreat
7. **return home** - Move to ready state

## Viewing the Task Tree

To see the task stages in RViz:
1. Go to **Panels → Add New Panel**
2. Select **Motion Planning Tasks**
3. The task tree will show all stages and their status

## Object Details

- **Shape**: Cylinder
- **Position**: `(x=0.3, y=0.0, z=0.0)` in world frame
- **Height**: 0.1 meters
- **Diameter**: 0.02 meters

## Files

- [`src/mtc_node.cpp`](src/mtc_node.cpp) - Main MTC task implementation
- [`launch/mtc_demo.launch.py`](launch/mtc_demo.launch.py) - Launch file

## Dependencies

- moveit_task_constructor_core
- moveit_task_constructor_msgs
- moveit_ros_planning_interface
- rclcpp

## Learning Objectives

✅ Understanding MTC task stages and containers  
✅ Creating grasp and place pose generators  
✅ Managing object attachment/detachment  
✅ Configuring collision allowances  
✅ Composing complex manipulation tasks  
✅ Using Cartesian and sampling planners

## Customization

Modify the following in `mtc_node.cpp` to customize behavior:

- **Object position**: Line 60-62 in `setupPlanningScene()`
- **Grasp offset**: Line 182 in grasp pose generation
- **Place offset**: Line 262 in place pose generation
- **Approach distance**: Lines 163, 224, 299 in `setMinMaxDistance()`
- **Planning timeout**: Lines 144, 242 in `setTimeout()`

## Troubleshooting

If planning fails:
- Check that the `ready` state is defined in the SRDF
- Verify joint limits allow reaching the object
- Ensure the object is within the robot's workspace
- Increase planning timeout if needed
