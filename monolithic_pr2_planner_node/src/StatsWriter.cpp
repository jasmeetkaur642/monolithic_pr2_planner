#include <monolithic_pr2_planner_node/StatsWriter.h>
#include <stdio.h>
#include <sstream>

using namespace std;
using namespace monolithic_pr2_planner;

StatsWriter::StatsWriter(){ }

void StatsWriter::writeRRT(int trial_id, RRTData data){
    ROS_INFO("writing rrt stats");
    stringstream ss;
    ss << "/tmp/planning_stats/rrt_" << trial_id << ".stats";
    FILE* stats = fopen(ss.str().c_str(), "w");
    if (data.planned){
        fprintf(stats, "%f %f\n", data.plan_time, data.shortcut_time);
        stringstream ss2;
        ss2 << "/tmp/planning_stats/rrt_" << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < data.robot_state.size(); i++){
            vector<double> l_arm;
            vector<double> r_arm;
            vector<double> base;

            data.robot_state[i].right_arm().getAngles(&r_arm);
            data.robot_state[i].left_arm().getAngles(&l_arm);

            fprintf(path, "%f %f %f %f %f %f %f\n",
                          r_arm[0], 
                          r_arm[1], 
                          r_arm[2], 
                          r_arm[3], 
                          r_arm[4], 
                          r_arm[5], 
                          r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f %f\n",
                          l_arm[0], 
                          l_arm[1], 
                          l_arm[2], 
                          l_arm[3], 
                          l_arm[4], 
                          l_arm[5], 
                          l_arm[6]);
            fprintf(path, "%f %f %f %f\n",
                          data.base[i].x(),
                          data.base[i].y(),
                          data.base[i].z(),
                          data.base[i].theta());
        }
        fclose(path);
    } else {
        fprintf(stats, "failed to plan\n");
    }
    fclose(stats);
}

void StatsWriter::writeARA(std::vector<double> &stats_v, std::vector<FullBodyState> &states, 
                           int trial_id){
    ROS_INFO("writing ara stats");
    stringstream ss;
    ss << "/tmp/planning_stats/ara_" << trial_id << ".stats";
    FILE* stats = fopen(ss.str().c_str(), "w");
    fprintf(stats, "%f %f %f %f %f %f %f %f %f %f\n", stats_v[0],
            stats_v[1],
            stats_v[2],
            stats_v[3],
            stats_v[4],
            stats_v[5],
            stats_v[6],
            stats_v[7],
            stats_v[8],
            stats_v[9]);
    stringstream ss2;

    if (states.size()){
        ss2 << "/tmp/planning_stats/ara_" << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < states.size(); i++){
            vector<double> l_arm = states[i].left_arm;
            vector<double> r_arm = states[i].right_arm;
            vector<double> base = states[i].base;

            fprintf(path, "%f %f %f %f %f %f %f\n",
                    r_arm[0], 
                    r_arm[1], 
                    r_arm[2], 
                    r_arm[3], 
                    r_arm[4], 
                    r_arm[5], 
                    r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f %f\n",
                    l_arm[0], 
                    l_arm[1], 
                    l_arm[2], 
                    l_arm[3], 
                    l_arm[4], 
                    l_arm[5], 
                    l_arm[6]);
            fprintf(path, "%f %f %f %f\n",
                    base[0],
                    base[1],
                    base[2],
                    base[3]);
        }
        fclose(path);
    }
    fclose(stats);
}

void StatsWriter::writeMHA(std::vector<double> &stats_v, std::vector<FullBodyState> &states, 
                           int trial_id){
    ROS_INFO("writing MHA stats");
    ROS_INFO("SIze of stats_v: %d", static_cast<int>(stats_v.size()));
    stringstream ss;
    ss << "/tmp/planning_stats/mha_" << trial_id << ".stats";
    FILE* stats = fopen(ss.str().c_str(), "w");
    fprintf(stats, "%f %f %f %f %f %f %f %f %f %f\n", stats_v[0],
            stats_v[1],
            stats_v[2],
            stats_v[3],
            stats_v[4],
            stats_v[5],
            stats_v[6],
            stats_v[7],
            stats_v[8],
            stats_v[9]);
    stringstream ss2;

    if (states.size()){
        ss2 << "/tmp/planning_stats/mha_" << trial_id << ".path";
        FILE* path = fopen(ss2.str().c_str(), "w");
        for (size_t i=0; i < states.size(); i++){
            vector<double> l_arm = states[i].left_arm;
            vector<double> r_arm = states[i].right_arm;
            vector<double> base = states[i].base;

            fprintf(path, "%f %f %f %f %f %f %f\n",
                    r_arm[0], 
                    r_arm[1], 
                    r_arm[2], 
                    r_arm[3], 
                    r_arm[4], 
                    r_arm[5], 
                    r_arm[6]);
            fprintf(path, "%f %f %f %f %f %f %f\n",
                    l_arm[0], 
                    l_arm[1], 
                    l_arm[2], 
                    l_arm[3], 
                    l_arm[4], 
                    l_arm[5], 
                    l_arm[6]);
            fprintf(path, "%f %f %f %f\n",
                    base[0],
                    base[1],
                    base[2],
                    base[3]);
        }
        fclose(path);
    }
    fclose(stats);
}

// void StatsWriter::writeEnvt(std::vector<monolithic_pr2_planner::Region> goal_regions,
//           monolithic_pr2_planner::RobotState start,
//           monolithic_pr2_planner::RobotState goal,
//           int trial_id){
//     ROS_INFO("writing Envt stats");
//     stringstream ss;
//     ss << "/tmp/planning_stats/envt_" << trial_id << ".stats";
//     FILE* stats = fopen(ss.str().c_str(), "w");
//     fprintf(stats, "%d\n", static_cast<int>(goal_regions.size()));
//     for (size_t i = 0; i < goal_regions.size(); ++i)
//     {
//         fprintf(stats, "%f %f %f %f %f %f\n",
//           goal_regions[i].x_min,
//           goal_regions[i].x_max,
//           goal_regions[i].y_min,
//           goal_regions[i].y_max,
//           goal_regions[i].z_min,
//           goal_regions[i].z_max);
//     }
//     for (size_t i=0; i < 2; i++){
//         vector<double> l_arm;
//         vector<double> r_arm;
//         vector<double> base;

//         switch(i){
//           case 0:
//             start.right_arm().getAngles(&r_arm);
//             start.left_arm().getAngles(&l_arm);
//           case 1:
//             goal.right_arm().getAngles(&r_arm);
//             goal.left_arm().getAngles(&l_arm);
//         }

//         fprintf(stats, "%f %f %f %f %f %f %f\n",
//                       r_arm[0], 
//                       r_arm[1], 
//                       r_arm[2], 
//                       r_arm[3], 
//                       r_arm[4], 
//                       r_arm[5], 
//                       r_arm[6]);
//         fprintf(stats, "%f %f %f %f %f %f %f\n",
//                       l_arm[0], 
//                       l_arm[1], 
//                       l_arm[2], 
//                       l_arm[3], 
//                       l_arm[4], 
//                       l_arm[5], 
//                       l_arm[6]);
//         // fprintf(stats, "%f %f %f %f\n",
//         //               start.base[i].x(),
//         //               start.base[i].y(),
//         //               start.base[i].z(),
//         //               start.base[i].theta());
//     }
//     fclose(stats);
// }