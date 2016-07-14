#include <monolithic_pr2_planner/Heuristics/MetricHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

// mode = 1 for arm
// mode = 2 for yaw
MetricHeuristic::MetricHeuristic(GoalState& goal_state, int mode, int cost_multiplier){
    m_cost_multiplier = cost_multiplier;
    if(mode == 1) {
        armMetric = true;
        yawMetric = false;
    }
    else if(mode == 2) {
        yawMetric = true;
        armMetric = false;
    }
    else {
        armMetric = false;
        yawMetric = false;
    }
    setGoal(goal_state);
}

MetricHeuristic::~MetricHeuristic(){}


void MetricHeuristic::setGoal(GoalState& goal_state){
    // Save the goal for future use.
    m_goal = goal_state;

    if(yawMetric)
        m_yaw = m_goal.getSolnState()->robot_pose().getContBaseState().theta();

    else if(armMetric) {
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getShoulderPanAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getShoulderLiftAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getUpperArmRollAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getElbowFlexAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getForearmRollAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getWristFlexAngle());
        goal_right_arm_angles.push_back(m_goal.getSolnState()->robot_pose().right_arm().getWristRollAngle());
        ROS_ERROR("Printing island arm");
        for(int i=0;i<7;i++)
            ROS_INFO("%f\t", goal_right_arm_angles[i]);
    }

    else
        ROS_ERROR("Metric Heuristic not recognised");
}


int MetricHeuristic::getGoalHeuristic(GraphStatePtr state){
    int cost = 0;
    double cost_ = 0;

    if(armMetric) {

        std::vector<double> current_right_arm_angles;
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getShoulderPanAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getShoulderLiftAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getUpperArmRollAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getElbowFlexAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getForearmRollAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getWristFlexAngle());
        current_right_arm_angles.push_back(state->robot_pose().right_arm().getWristRollAngle());

        for(int i=0;i<7;i++) {
            cost_ += m_cost_multiplier*((current_right_arm_angles[i] - goal_right_arm_angles[i]) * (current_right_arm_angles[i] - goal_right_arm_angles[i]));
        }
        cost = int(cost_);
    }

    else if(yawMetric) {
        cost_ = m_cost_multiplier*((m_yaw - state->robot_pose().getContBaseState().theta()) *(m_yaw - state->robot_pose().getContBaseState().theta())); 
        cost = int(cost_);
    }

    else ROS_ERROR("Metric Heuristic mode not recognised");
    
    return cost;
}
