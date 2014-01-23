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
    class BFS2DHeuristic : public AbstractHeuristic, public OccupancyGridUser{
        public:
            BFS2DHeuristic();
            // BFS2DHeuristic(boost::shared_ptr<costmap_2d::Costmap2DROS>
                // costmap_ros);
            ~BFS2DHeuristic();
            void setGoal(GoalState& state);
            void loadMap(const std::vector<signed char>& data);
            int getGoalHeuristic(GraphStatePtr state);
            void update2DHeuristicMap(const std::vector<signed char>& data);
            void visualizeCenter(int x, int y);

            // Set radius for the circle around the goal for which the
            // heuristic will be zero. Default is the PR2's arm reach
            // TODO: Compute this instead of hardcoding
            void setRadiusAroundGoal(double radius_m = 0.7);
            double getRadiusAroundGoal(){ return m_radius; };
        private:
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
            double m_radius;
            void visualizeRadiusAroundGoal(int x0, int y0);
            // ros::NodeHandle m_nh;
            // ros::Publisher m_circlepub;

    };
    typedef boost::shared_ptr<BFS2DHeuristic> BaseHeuristicPtr;
}