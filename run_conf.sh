#!/bin/bash
export ROS_DOMAIN_ID=4
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

source ~/ros2_jazzy/install/setup.bash
ros2 node list

channel_num=5
periods=(100 90 80 70 60 50)
jitter=1.2
date

worst_periods_str=""
for ((i=0; i<channel_num; i++)); do
    wp=$(echo "${periods[$i]} * $jitter" | bc -l)
    # 追加逗号分隔
    if [[ $i -ne 0 ]]; then
        worst_periods_str+=","
    fi
    worst_periods_str+="$wp"
    # echo "worst_periods_str=$worst_periods_str"
done

# c_bound=$(python3 -c "import sys; sys.path.insert(0, 'scripts'); from latency_calculation import cal_ros_bound; print(cal_ros_bound([${worst_periods_str}]))")
# max_period=$(python3 -c "print(max([${worst_periods_str}]))")
# between=$(python3 -c "print($c_bound+($max_period - $c_bound)/2)")
# low=$(python3 -c "print($c_bound-($max_period - $c_bound)/2)")
# high=$(python3 -c "print($max_period+($max_period - $c_bound)/2)")

c_bound_float=$(python3 -c "import sys; sys.path.insert(0, 'scripts'); from latency_calculation import cal_ros_bound; print(cal_ros_bound([${worst_periods_str}]))")
c_bound=$(printf "%.0f" "$c_bound_float")

max_period_float=$(python3 -c "print(max([${worst_periods_str}]))")
max_period=$(printf "%.0f" "$max_period_float")

between=$(( c_bound + (max_period - c_bound) / 2 ))
low=$(( c_bound - (max_period - c_bound) / 2 ))
high=$(( max_period + (max_period - c_bound) / 2 ))

# echo "worst_periods_str=$worst_periods_str"
# echo "c_bound=$c_bound"
# echo "between=$between"
# echo "结果是：$c_bound"
# echo "最大值是: $max_period"


for i in {1..1}; do
    for c in $low $c_bound $between $max_period $high; do
    # for c in 60 90; do
        echo "Running test with c=$c"
        ros2 run seam_analysis seam_config $channel_num $c ${periods[0]} ${periods[1]} ${periods[2]} ${periods[3]} ${periods[4]} ${periods[5]}  /home/rongshun/ros2_jazzy/src/seam_analysis/results/config_$ROS_DOMAIN_ID/$c --ros-args --params-file /home/rongshun/ros2_jazzy/src/seam_analysis/config/config.yaml
    done
done
echo "All tests completed at:"
date

exit 0
