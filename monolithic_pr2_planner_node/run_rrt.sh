#!/bin/bash

# Set the right path to testset.
# Set the right path to where stats are to be copied.


#the planner expects the environment to stored at tableObstacles.yaml
#so move the numbered one to this path
#cp experiments/tableObstacles200.yaml experiments/tableObstacles.yaml

#for method in unconstrained_mha focal_mha mha_plus original_mha; do
method="rrt"

#for experiment_num in 1 2 3;do
for experiment_num in 1;do
    rm -r /tmp/planning_stats/*
    cp experiments/experiment${experiment_num}/tableObstacles.yaml experiments/tableObstacles.yaml
    #start the planner
    echo "Starting planner";
    screen -dmS environment bash -c "source /opt/ros/indigo/setup.bash; source /home/aries/ROS/catkin_ws/devel/setup.bash;roslaunch monolithic_pr2_planner_node launch_environment.launch; exec bash"
    #a hack because the planner takes a while to start
    sleep 80 #50


    screen -dmS planner bash -c "source /opt/ros/indigo/setup.bash; source /home/aries/ROS/catkin_ws/devel/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments.launch; exec bash"

    sleep 20

    #send the tests
    echo "Sending goals from environment using the $method method"

    echo "Calling planner"
    rosrun monolithic_pr2_planner_node runTests "$method" "experiments/experiment${experiment_num}/fbp${experiment_num}_testset.yaml"


    #save the stats
    statsFileName="$(date +%d%b)"
    #eps="20"
    statsFileName="${statsFileName}_rrt_environment_${experiment_num}.csv"
    echo "$statsFileName"


    sleep 5

    mkdir -p "stats_collection/rrt/experiment${experiment_num}/test/paths_${statsFileName}"
    mv /tmp/planning_stats/* "stats_collection/rrt/experiment${experiment_num}/test/paths_${statsFileName}"

    mkdir -p "stats_collection/rrt/experiment${experiment_num}/test"
    cp mha_stats.csv "stats_collection/rrt/experiment${experiment_num}/test/${statsFileName}"

    sleep 2

    screen -S planner -X quit
    screen -S environment -X quit
done
