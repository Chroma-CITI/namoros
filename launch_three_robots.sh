#!/bin/bash

DIR="$(dirname "$(readlink -f "$0")")"
cd ${DIR}

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:${DIR}/namoros/config

source ./install/setup.bash 
ros2 run namoros scenario2sdf --svg-file=${DIR}/namoros/config/three_robots.svg --out-dir=${DIR}/namoros/config
colcon build
ros2 launch namoros launch.sim.py \
    scenario_file:=${DIR}/namoros/config/three_robots.svg \
    config_file:=${DIR}/namoros/config/namoros_config.yaml \
    sdf_file:=${DIR}/namoros/config/three_robots.sdf \
    map_yaml:=${DIR}/namoros/config/three_robots.yaml \
    omniscient_obstacle_perception:="true"