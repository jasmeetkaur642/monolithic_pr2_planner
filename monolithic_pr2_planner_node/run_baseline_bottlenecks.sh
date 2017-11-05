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
        # 1 => 10 heuristics and 2 => 20 heuristics
        heuristic=2
        heur="20"
        rm -r /tmp/planning_stats/*

        screen -dmS planner bash -c "source /opt/ros/indigo/setup.bash; source /home/aries/ROS/catkin_ws/devel/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments.launch; exec bash"

        sleep 20

        rosparam set "/heuristic_set_type" "$heuristic"
        rosparam set "/monolithic_experiment/eps" "$eps"


        echo "$heuristic heuristic being used."

        #send the tests
        echo "Sending goals from environment using the $method method"

        echo "Calling planner"
        rosrun monolithic_pr2_planner_node runTests smha "$method" "experiments/experiment${experiment_num}/fbp${experiment_num}_testset.yaml"


        #save the stats
        statsFileName="$(date +%d%b)"
        #eps="20"
        statsFileName="baseline_rr_heuristics_${heur}_eps_${eps}.csv"
        echo "$statsFileName"


        sleep 5

        mkdir -p "stats_collection/rr/experiment${experiment_num}/test/paths_${statsFileName}"
        mv /tmp/planning_stats/* "stats_collection/rr/experiment${experiment_num}/test/paths_${statsFileName}"

        #mkdir -p "stats_collection/rr/experiment2/test/baseline_paths_${method}_${heur}_${eps}"
        #mv /tmp/planning_stats/* "stats_collection/rr/experiment2/test/baselin_paths_${method}_${heur}_${eps}"

        mkdir -p "stats_collection/rr/experiment${experiment_num}/test"
        cp mha_stats.csv "stats_collection/rr/experiment${experiment_num}/test/${statsFileName}"

        sleep 2

        screen -S planner -X quit
    done
    screen -S environment -X quit
done

