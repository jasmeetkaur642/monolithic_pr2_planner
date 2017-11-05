#!/bin/bash

# Set the right path to testset.
# Set the right path to where stats are to be copied.

rm /home/aries/ROS/catkin_ws/src/monolithic_pr2_planner/monolithic_pr2_planner_node/snapMprimStats.txt

#the planner expects the environment to stored at tableObstacles.yaml
#so move the numbered one to this path
#cp experiments/tableObstacles200.yaml experiments/tableObstacles.yaml

#for method in unconstrained_mha focal_mha mha_plus original_mha; do
method="rr_focal"

for experiment_num in 1 2;do
    cp experiments/experiment${experiment_num}/tableObstacles.yaml experiments/tableObstacles.yaml
    #start the planner
    echo "Starting planner";
    screen -dmS environment bash -c "source /opt/ros/indigo/setup.bash; source /home/aries/ROS/catkin_ws/devel/setup.bash;roslaunch monolithic_pr2_planner_node launch_environment.launch; exec bash"
    #a hack because the planner takes a while to start
    sleep 80 #50

    #for eps in 20 50; do
    for eps in 50;do
        heur="20"
        heuristic=2
        #experiment_num=1
        rm -r /tmp/planning_stats/*
        rm /home/aries/.ros/state_time.csv

        screen -dmS planner bash -c "source /opt/ros/indigo/setup.bash; source /home/aries/ROS/catkin_ws/devel/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments.launch; exec bash"

        sleep 20

        # 1 => 10 heuristics and 2 => 20 heuristics
        rosparam set "/heuristic_set_type" "$heuristic"
        rosparam set "/monolithic_experiment/eps" "$eps"

        heuristic=($heuristic*10)

        echo "$heuristic heuristic being used."

        #send the tests
        echo "Sending goals from environment using the $method method"

        echo "Calling planner"
        rosrun monolithic_pr2_planner_node runTests smha "$method" "experiments/experiment${experiment_num}/fbp${experiment_num}_trainingset.yaml"


        #save the stats
        statsFileName="training_rr_heuristics_${heur}_eps_${eps}.csv"
        echo "$statsFileName"

        sleep 5

        cp mha_stats.csv "stats_collection/rr/experiment${experiment_num}/${statsFileName}"
        mv "/home/aries/.ros/state_time.csv" "stats_collection/rr/experiment${experiment_num}/state_time_rr_${heur}_${eps}.csv"

        sleep 2

        screen -S planner -X quit
    done

    done

screen -S environment -X quit
