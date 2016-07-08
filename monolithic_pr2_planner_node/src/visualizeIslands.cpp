#include <monolithic_pr2_planner_node/interactFullBodyPlanner.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner_node/fbp_stat_writer.h>
#include <sbpl/planners/mha_planner.h>

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <pr2_collision_checker/body_pose.h>
#include <vector>
#include <pviz/pviz.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualize_islands");
    ros::NodeHandle nh("~");
    PViz pviz;

    int num_islands;
    string island_file_name;

    nh.param("island_file_name", island_file_name, std::string(" "));
    nh.param("num_islands", num_islands, 0);
    ROS_INFO("Loaded island parameters %s, %d",  island_file_name.c_str(), num_islands);

    bool baseIslandHeur = false;
    bool armIslandHeur = true;
    std::vector<double> base = {0, 0, 0};
    double torso_z = 0.3;
    std::vector<double> left_angles;
    left_angles.resize(7);

    left_angles[0] = -0.002109;
    left_angles[1] = 0.655300;
    left_angles[2] = 0.000000;
    left_angles[3] = -1.517650;
    left_angles[4] = -3.138816;
    left_angles[5] = -0.862352;
    left_angles[6] = 3.139786;

    ROS_ERROR("HERE1");
    ifstream island_file(island_file_name);
    ROS_ERROR("%s", island_file_name.c_str());
    std::string line;

    if(baseIslandHeur) {
        for(int i=0;i<num_islands;i++) {
            ROS_ERROR("init baseIslandBase %d", i);
            std::getline(island_file, line);
            ROS_ERROR("%s", line.c_str());
            std::istringstream ss(line);
            std::string x_str, y_str;

            ss >> x_str >> y_str;

            double x=0, y=0;
            x = atof(x_str.c_str());
            y = atof(y_str.c_str());
            ROS_ERROR("Island points %f %f", x, y);
            //addBaseIslandHeur("baseIslandBase" + std::to_string(i), 1, sbpl_2Dpt_t(x, y), radius_around_goal);
        }
    }
    else if(armIslandHeur) {
    ROS_ERROR("HERE2");
        for(int i=0;i<num_islands;i++) {
            getchar();
            std::getline(island_file, line);
            ROS_ERROR("%s", line.c_str());
            std::istringstream ss(line);
            std::string num;
            std::vector<double> arm_angles;

            while(ss >> num)
                arm_angles.push_back(atof(num.c_str()));

            pviz.visualizeRobot(arm_angles, left_angles, base, torso_z, 127, "huehuehue", 0, false);
        }
    }

    ROS_ERROR("Visualized all islands");

}
