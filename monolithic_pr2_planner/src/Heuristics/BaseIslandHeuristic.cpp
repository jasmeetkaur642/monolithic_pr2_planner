#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/cost_values.h>

using namespace monolithic_pr2_planner;

BaseIslandHeuristic::BaseIslandHeuristic() {
    
    // Simply the distance of current state from the island/
}
