# NAMOROS

A set of ROS2 nodes which expose services and topics for computing NAMO plans and interacting with the **namosim** planner
This repo consists of the following ROS2 packages:

## System Requirements

- Ubuntu 22.04
- ROS2 Humble

## Setup

First, clone the repo.

```bash
git clone --recurse-submodules git@gitlab.inria.fr:chroma/namo/namoros.git
cd namoros
```

Next, use `rosdep` to install the dependencies listed in the `package.xml` files:

```bash
rosdep install --from-paths . -r -y
```

If any of the python dependencies fail to install with `rosdep` you can try to install them with `pip` instead:

```bash
pip install -r namosim/requirements.txt
pip install -r namoros/requirements.txt
```

Finally, build and source the project.

```bash
colcon build
source install/setup.bash
```

## Demostrations

In order to launch a simulation demonstration a robot that makes use of namoros nodes, run the following script.

```bash
./namoros/launch_demo.sh
```

## Architecture

The system is organized as ROS2 packages:

- `namoros`: ROS2 nodes to control the robot and interacting with the namosim planner within a behavior tree framework
- `namosim`: The core planner for navigation and multi-robot coordination
- `namoros_msgs`: Custom ROS2 message definitions
- `namoros_gz`: Custom Gazebo plugin for simulating grab and release actions.

Here is a block diagram showing the main components of NAMOROS:

<img src="static/NAMOROS_Architecture.png" alt="NAMOROS Architecturelabel">

The **NAMO Planner** block is a custom ROS2 node that manages the namosim planner and exposes services and actions for interacting with it.

The **NAMO Behavior Tree** block is another custom node that executes that main behavior tree which controls the robot execution and interaction with the planner node.

The other blocks, `nav2` and `aruco_markers` are third-party packages used for simple navigation and detection of visual markers placed on movable obstacles.

If running in a Gazebo simulation, a plugin from the `namoros_gz` package is provided to simulate grab and release actions. It works by dynamically creating a fixed joint between a user-chosen link on the robot and a link on the obstacle.

### Main Behavior Tree

The main behavior tree is illustrated in the following diagram. It ticks at a frequency of 2Hz. The robot starts by waiting to receive a start pose and goal pose. These may come from the scenario file or be published to the corresponding topics. The behavior tree continuously motors its sensor data for the positions of other robots and movable obstacles. It uses this data during specific periods to synchronize the planner node's state with the estimated state of the environment which is necessary for conflict detection. The _New Movable_ node encapsulates a subtree that handles dynamic detection of movable obstacles but is only used when that feature is activated and not shown for brevity.

<img src="static/namo_main_tree.svg" alt="Main Behavior Tree">

### Execute Plan Subtree

Because a NAMO plan consists of multiple behaviors such as path following, and grabbing and releasing obstacles, and because the plan is initially unknown and subject to change, the _Execute Plan_ bevavior dynamically creates and executes a subtree corresponding to the current plan. The following diagram shows an example subtree which consists of a _transit_ path followed by a _transfer_ path to move an obstacle, and lastly another transit path to reach the goal. Immediately before and after each _transfer_ path there are also grab and release sequences. Each of these behaviors are themselves small subtrees which are illustrated below. The _Execute Plan_ subtree always starts with a release behavior to just in case the robot was already holding an obstacle at the time the plan was computed.

<img src="static/execute_plan_tree.svg" alt="Execute Plan Tree" width="800">

### Transit Path

The transit path uses nav2 to follow the corresponding path segment within the NAMO plan.

<img src="static/transit_path_tree.svg" alt="Transit Path Tree" width="700">

### Grab Sequence

The grab sequence consists of first turning towards the obstacle, approaching it within close range as determined by the lidar sensor, and finally performing the grab action. An important point is that synchronizing the planner with the observed obstacle state is disabled
because the planner treats robot and the obstacle as a single object during transfer paths. Otherwise, the planner will detect conflicts with the obstacle the robot is already carrying.

<img src="static/grab_tree.svg" alt="Grab Tree" width="700">

### Release Sequence

The release sequence first performs the release action, and the backs the robot up at a constant slow speed for a fixed time period. Then the robot re-estimates the obstacle position, synchronizes the planner, and re-computes the plan.

<img src="static/release_tree.svg" alt="Release Tree" width="700">

## Conflict Handling

During path following, the behavior tree periodically synchronizes the planner node with the current estimated state of the environment and checks for conflicts. When a conflict is detected, the robot is interrupted, the plan is _updated_, and then plan execution is restarted.

## Authors

- David Brown
- Jacques Saraydaryan
- Olivier Simonin
- Benoit Renault

## Affiliated Teams and Organisations

|                                    | Org/Team                                      |
| ---------------------------------- | --------------------------------------------- |
| ![Inria Logo](static/inria.png)    | [Inria](https://inria.fr/fr)                  |
| ![INSA Lyon Logo](static/insa.png) | [INSA Lyon](https://www.insa-lyon.fr/)        |
| ![CITI Logo](static/citi.png)      | [CITI Laboratory](https://www.citi-lab.fr/)   |
| CHROMA                             | [CHROMA Team](https://www.inria.fr/en/chroma) |

## Cite Us

If you reuse any of the provided data/code, please cite the associated papers:

```bibtex
@inproceedings{renault:hal-04705395,
  TITLE = {{Multi-Robot Navigation among Movable Obstacles: Implicit Coordination to Deal with Conflicts and Deadlocks}},
  AUTHOR = {Renault, Benoit and Saraydaryan, Jacques and Brown, David and Simonin, Olivier},
  URL = {https://hal.science/hal-04705395},
  BOOKTITLE = {{IROS 2024 - IEEE/RSJ International Conference on Intelligent Robots and Systems}},
  ADDRESS = {Abu DHABI, United Arab Emirates},
  PUBLISHER = {{IEEE}},
  PAGES = {1-7},
  YEAR = {2024},
  MONTH = Oct,
  KEYWORDS = {Planning ; Scheduling and Coordination ; Path Planning for Multiple Mobile Robots or Agents ; Multi-Robot Systems},
  PDF = {https://hal.science/hal-04705395v1/file/IROS24_1134_FI.pdf},
  HAL_ID = {hal-04705395},
  HAL_VERSION = {v1},
}
```

```bibtex
@inproceedings{renault:hal-02912925,
  TITLE = {{Modeling a Social Placement Cost to Extend Navigation Among Movable Obstacles (NAMO) Algorithms}},
  AUTHOR = {Renault, Benoit and Saraydaryan, Jacques and Simonin, Olivier},
  URL = {https://hal.archives-ouvertes.fr/hal-02912925},
  BOOKTITLE = {{IROS 2020 - IEEE/RSJ International Conference on Intelligent Robots and Systems}},
  ADDRESS = {Las Vegas, United States},
  SERIES = {2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) Conference Proceedings},
  PAGES = {11345-11351},
  YEAR = {2020},
  MONTH = Oct,
  DOI = {10.1109/IROS45743.2020.9340892},
  KEYWORDS = {Navigation Among Movable Obstacles (NAMO) ; Socially- Aware Navigation (SAN) ; Path planning ; Simulation},
  PDF = {https://hal.archives-ouvertes.fr/hal-02912925/file/IROS_2020_Camera_Ready.pdf},
  HAL_ID = {hal-02912925},
  HAL_VERSION = {v1},
}
```
