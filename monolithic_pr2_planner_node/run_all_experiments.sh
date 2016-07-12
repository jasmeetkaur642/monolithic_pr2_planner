#!/bin/bash

rm -r /tmp/planning_stats/*

folder_name="arm_islands"

for i in 200; do
  #the planner expects the environment to stored at tableObstacles.yaml
  #so move the numbered one to this path
  cp experiments/tableObstacles$i.yaml experiments/tableObstacles.yaml

  #for method in unconstrained_mha focal_mha mha_plus original_mha; do
  for method in focal_mha; do
  #for method in gbfs; do

    #create necessary folders
    mkdir -p stats
    mkdir -p stats_collection/arm_islands/paths_${method}_${i}

    echo "Starting projected map saver"
    screen -dmS saver bash -c 'source /opt/ros/indigo/setup.bash; source \
    /home/shivam/ROS/indigo_ws/devel/setup.bash; rosrun monolithic_pr2_planner_node \
    saveProjectedMap \
    "/home/shivam/ROS/indigo_ws/src/monolithic_pr2_planner/monolithic_pr2_planner_node/stats_collection/$0"; exec bash' $folder_name

    #start the planner
    echo "starting planner";
    screen -dmS planner bash -c "source /opt/ros/indigo/setup.bash; source /home/shivam/ROS/indigo_ws/devel/setup.bash; roslaunch monolithic_pr2_planner_node run_experiments_islands.launch; exec bash" 
    #a hack because the planner takes a while to start
    sleep 60 #50

    echo "Killing saver"
    screen -S saver -X quit

    #send the tests
    echo "sending goals from environment $i using the $method method"

    if [ $method == "rrt" ]; then
      rosrun monolithic_pr2_planner_node runTests experiments/fbp_tests$i.yaml $method
    else
        echo "Calling planner"
      rosrun monolithic_pr2_planner_node runTests experiments/fbp_tests$i.yaml smha $method
    fi

    #when the trials are done, kill the planner
    #screen -r planner
    echo "killing planner"
    screen -S planner -X quit
    #wait a bit to make sure the planner died
    sleep 30

    #save the stats
    #mv mha_stats.csv stats/mha_stats_${method}_${i}.csv
    #mv /tmp/planning_stats/* stats/paths_${method}_${i}
    mv mha_stats.csv stats_collection/$folder_name/mha_stats_${method}_${i}.csv
    mv /tmp/planning_stats/* stats_collection/$folder_name/paths_${method}_${i}
  done
done
