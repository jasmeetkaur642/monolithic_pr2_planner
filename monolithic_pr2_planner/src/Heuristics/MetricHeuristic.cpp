#include <monolithic_pr2_planner/Heuristics/MetricHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

MetricHeuristic::MetricHeuristic(GoalState& goal_state){
    setGoal(goal_state);
}

MetricHeuristic::~MetricHeuristic(){}


void MetricHeuristic::setGoal(GoalState& goal_state){
    // Save the goal for future use.
    m_goal = goal_state;

    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getShoulderPanAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getShoulderLiftAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getUpperArmRollAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getElbowFlexAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getForearmRollAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getWristFlexAngle());
    goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getWristRollAngle());
}

int MetricHeuristic::getGoalHeuristic(GraphStatePtr state){
    std::vector<double> current_right_arm_angles;
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getShoulderPanAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getShoulderLiftAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getUpperArmRollAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getElbowFlexAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getForearmRollAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getWristFlexAngle());
    current_right_arm_angles.push_back(state->robot_pose().right_arm().getWristRollAngle());
    
    int cost = 0;

    for(int i=0;i < 7;i++) {
         cost += (goal_right_arm_angles[i] - current_right_arm_angles[i])*(goal_right_arm_angles[i] - current_right_arm_angles[i]);
    }
    //ROS_INFO("Island Heuristic cost = %d", cost);
    
    return cost;
}
