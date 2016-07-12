#pragma once
// #include <sbpl/headers.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/Heuristics/2Dgridsearch.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class MetricHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser {
        public:
            MetricHeuristic() {}
            MetricHeuristic(GoalState& goal_state);
            ~MetricHeuristic();
            
            void setGoal(GoalState& state);
            //void loadMap(const std::vector<unsigned char>& data);
            int getGoalHeuristic(GraphStatePtr state);
        protected:
            GoalState m_goal;
            std::vector<double> goal_right_arm_angles;
            std::vector<double> goal_left_arm_angles;
            double m_yaw;


    };
    typedef boost::shared_ptr<MetricHeuristic> MetricHeuristicPtr;
}
