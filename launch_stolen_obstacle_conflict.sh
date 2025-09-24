#!/bin/bash

DIR="$(dirname "$(readlink -f "$0")")"
cd ${DIR}

colcon build
source ./install/setup.bash 
ros2 run namoros scenario2sdf --svg-file=${DIR}/namoros/config/stolen_obstacle_conflict.svg --out-dir=${DIR}/namoros/config
colcon build
ros2 launch namoros launch.sim.py \
    scenario_file:=${DIR}/namoros/config/stolen_obstacle_conflict.svg \
    config_file:=${DIR}/namoros/config/namoros_config.yaml \
    sdf_file:=${DIR}/namoros/config/namo_world.sdf \
    map_yaml:=${DIR}/namoros/config/stolen_obstacle_conflict.yaml \
    omniscient_obstacle_perception:="true" \
    autostart:="true"