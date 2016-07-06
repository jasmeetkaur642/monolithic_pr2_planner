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
#include <vector>
#include <utility>

namespace monolithic_pr2_planner {
    class BaseIslandHeuristic : public BFS2DHeuristic {
        public:
            BaseIslandHeuristic();
            ~BaseIslandHeuristic();
            void setGoal(GoalState& goal_state);
            int getGoalHeuristic(GraphStatePtr state);

        private:
            // Critical points/islands on the map, calculated after running
            // MeanShift.
            std::vector<std::pair<double, double> > islands;

};

typedef boost::shared_prt<BaseIslandHeuristic> BaseIslandHeuristicPtr;
