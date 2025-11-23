#!/bin/bash
export ROS_DOMAIN_ID=1
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
ros2 node list
source ~/ros2_jazzy/install/setup.bash
ros2 node list

test_num=100
c_bound=110
lower_limit=50
upper_limit=100
channel_num=6

for (( test_n=1; test_n<=$test_num; test_n++ )); do
    echo "Test number: $test_n"
    ######################### C (bound used in SEAM) ####################
    for c in {60..140..5}; do
        ros2 run seam_analysis seam $channel_num $lower_limit $upper_limit 1 $c /home/z786/ros2_jazzy/src/seam_analysis/results/c_bound/$c --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
    done

    ################################# Number of Channel #############################
    for ch in {3..9}; do
        ros2 run seam_analysis seam $ch $lower_limit $upper_limit 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/$ch --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
    done

    ######################## Lower Period (Range upper 100ms) ###########################
    for period in {10..50..10}; do
        ros2 run seam_analysis seam $channel_num $period $upper_limit 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/lower_period/$period --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
    done

    ######################### Upper Period (Range lower 50ms) ###########################
    for period in {80..120..10}; do
        ros2 run seam_analysis seam $channel_num $lower_limit $period 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/upper_period/$period --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
    done

    ################################ Jitter Ratio Tw/Tb ####################################
    for j in {0..8..2}; do
        ros2 run seam_analysis seam $channel_num $lower_limit $upper_limit 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/jitter/$j --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_${j}.yaml
    done

    ################################ Max Random Delay Dw ####################################
    for d in {0..40..10}; do
        ros2 run seam_analysis seam $channel_num $lower_limit $upper_limit 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/delay/$d --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_${d}.yaml
    done

    ################################ Different Channel ####################################
    for d in {0..5..1}; do
        ros2 run seam_analysis seam $channel_num $lower_limit $upper_limit 1 $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel/$d --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
    done

done
echo "All tests completed at:"
date

exit 0


# #!/bin/bash
# export ROS_DOMAIN_ID=1
# echo $ROS_DOMAIN_ID
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# echo $RMW_IMPLEMENTATION
# source ~/ros2_jazzy/install/setup.bash
# ros2 node list
# # Number of test group
# test_num=10
# c_bound=110
# lower_limit=50
# channel_num=6
# ########################## C (bound used in SEAM) ####################
# # ros2 run seam_analysis seam $channel_num $lower_limit $test_num 50 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_50 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# # ros2 run seam_analysis seam $channel_num $lower_limit $test_num 55 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_55 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 60 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_60 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 65 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_65 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 70 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_70 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 75 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_75 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 80 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_80 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 85 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_85 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 90 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_90 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 95 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_95 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 100 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_100 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 105 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_105 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 110 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_110 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 115 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_115 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 120 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_120 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 125 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_125 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 130 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_130 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 135 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_135 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num 140 /home/z786/ros2_jazzy/src/seam_analysis/results/C/C_140 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml

# ################################# Number of Channel #############################
# ros2 run seam_analysis seam 3 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_3 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 4 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_4 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 5 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_5 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 6 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_6 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 7 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_7 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 8 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_8 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam 9 $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/channel_num/channel_9 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml

# ######################### Period (Range upper 100ms) ###########################
# ros2 run seam_analysis seam $channel_num 10 $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/period/period_10 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num 20 $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/period/period_20 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num 30 $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/period/period_30 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num 40 $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/period/period_40 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml
# ros2 run seam_analysis seam $channel_num 50 $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/period/period_50 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_default.yaml

# ################################ Jitter Ratio Tw/Tb ####################################
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/J/J_0 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_0.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/J/J_2 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_2.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/J/J_4 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_4.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/J/J_6 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_6.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/J/J_8 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_J_8.yaml

# ################################ Max Random Delay Dw ####################################
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/D/D_0 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_0.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/D/D_10 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_10.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/D/D_20 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_20.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/D/D_30 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_30.yaml
# ros2 run seam_analysis seam $channel_num $lower_limit $test_num $c_bound /home/z786/ros2_jazzy/src/seam_analysis/results/D/D_40 --ros-args --params-file /home/z786/ros2_jazzy/src/seam_analysis/config/config_D_40.yaml


# wait

# date

# exit
