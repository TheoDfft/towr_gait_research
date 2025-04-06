# TOWR Hopper Example

This is a simple example of using TOWR with a monoped (hopper) robot in ROS. It demonstrates how to set up a trajectory optimization problem for a single-legged robot and visualize the results.

## Overview

The hopper example is based on the standalone example in `towr/test/hopper_example.cc` but has been integrated with ROS for visualization and interaction capabilities. This implementation optimizes for a jumping motion that makes the hopper reach a configurable height above its initial position.

The implementation includes:
- Custom ROS node (`HopperRosApp`) derived from the TOWR ROS interface
- Specialized configuration for the monoped robot model
- Dedicated launch file for easy startup
- Visualization in RViz

## Building

To build the hopper example, use the standard catkin build system:

```bash
cd ~/gait_ws
catkin_make_isolated
source devel/setup.bash
```

## Running the Example

To run the hopper example with default parameters:

```bash
roslaunch towr_ros hopper_ros.launch
```

To customize the jump height (e.g., jump to 2.0 times the initial height):

```bash
roslaunch towr_ros hopper_ros.launch height_multiplier:=2.0
```

The launch file will:
1. Start the TOWR hopper node with the specified parameters
2. Launch RViz with the appropriate configuration
3. Start visualization nodes for the robot and trajectories

## Jump Trajectory

By default, the hopper will optimize a trajectory to:
- Jump in place (no horizontal movement)
- Reach a height that is 1.5 times its initial height (from 0.5m to 0.75m)
- Follow a sequence of phases: initial stance, push-off, flight, landing, final stance

The trajectory is optimized to satisfy dynamic feasibility constraints while achieving the target height. The optimization ensures that:
- The foot lifts off the ground during the flight phase
- The motion respects the robot's dynamic capabilities
- Forces are applied appropriately during stance phases

## Parameters

You can customize the following parameters through the launch file:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `height_multiplier` | 1.5 | How much higher to jump compared to initial height |
| `optimize_durations` | true | Whether to optimize the phase durations |
| `debug` | false | Run with gdb for debugging |

Example:
```bash
roslaunch towr_ros hopper_ros.launch height_multiplier:=2.0 optimize_durations:=false
```

## Debugging

To run the application with gdb for debugging:

```bash
roslaunch towr_ros hopper_ros.launch debug:=true
```

## References

- Original TOWR paper: https://doi.org/10.1109/LRA.2018.2798285
- TOWR GitHub repository: https://github.com/ethz-adrl/towr 