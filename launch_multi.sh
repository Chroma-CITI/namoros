#!/bin/bash

DIR="$(dirname "$(readlink -f "$0")")"
cd ${DIR}

colcon build
source ./install/setup.bash 

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:${DIR}/namoros/config

ros2 run namoros scenario2sdf --svg-file=${DIR}/namoros/config/namoros_multi_map.svg --out-dir=${DIR}/namoros/config

ros2 launch namoros launch.sim.py \
    scenario_file:=${DIR}/namoros/config/namoros_multi_map.svg \
    config_file:=${DIR}/namoros/config/namoros_config.yaml \
    sdf_file:=${DIR}/namoros/config/namoros_multi_map.sdf \
    map_yaml:=${DIR}/namoros/config/namoros_multi_map.yaml \
    autostart:="true"