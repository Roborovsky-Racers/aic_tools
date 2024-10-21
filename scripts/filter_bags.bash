#!/bin/bash

BAGNAME_BASE="/aichallenge/workspace/rosbag2_autoware/rosbag2_2024_10_11-18_03_34_trim_"
FILTERED_BAGNAME_SUFFIX="_share"
TOPICS=(
    "/mpc/prediction"
    "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall"
    "/mpc/ref_path"
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound"
)
TOPIC_ARGS=$(printf " %s" "${TOPICS[@]}")

# trim_idx を 1~12 の範囲で実行
for idx in {1..12}
do
    echo "Executing: ros2 bag filter $BAGNAME_BASE$idx -o $BAGNAME_BASE$idx$FILTERED_BAGNAME_SUFFIX -x $TOPIC_ARGS"
    ros2 bag filter $BAGNAME_BASE$idx -o $BAGNAME_BASE$idx$FILTERED_BAGNAME_SUFFIX -x $TOPIC_ARGS
done
