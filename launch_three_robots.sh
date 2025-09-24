#!/bin/bash

DIR="$(dirname "$(readlink -f "$0")")"
cd ${DIR}

source ./install/setup.bash 
ros2 run namoros scenario2sdf --svg-file=${DIR}/namoros/config/three_robots.svg --out-dir=${DIR}/namoros/config
colcon build
ros2 launch namoros launch.sim.py \
    scenario_file:=${DIR}/namoros/config/three_robots.svg \
    config_file:=${DIR}/namoros/config/namoros_config.yaml \
    sdf_file:=${DIR}/namoros/config/namo_world.sdf \
    map_yaml:=${DIR}/namoros/config/three_robots.yaml \
    omniscient_obstacle_perception:="true"