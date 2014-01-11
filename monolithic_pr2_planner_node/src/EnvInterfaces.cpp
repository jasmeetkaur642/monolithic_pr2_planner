#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.
EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env) : 
    m_env(env), m_collision_space_interface(env->getCollisionSpace(), env->getHeuristicMgr()){
        getParams();
    bool forward_search = true;
    m_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    m_heur_map_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("heur_map", 1);
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<string>("reference_frame", m_params.ref_frame, 
                                    string("map"));
}

void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res){
    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->obj_goal= req.goal;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int start_id, goal_id;
    bool retVal = m_env->configureRequest(search_request, start_id, goal_id);
    if(!retVal){
        return false;
    }

    m_planner->set_initialsolution_eps(search_request->initial_epsilon);
    bool return_first_soln = true;
    m_planner->set_search_mode(return_first_soln);
    m_planner->set_start(start_id);
    ROS_INFO("setting goal id to %d", goal_id);
    m_planner->set_goal(goal_id);
    m_planner->force_planning_from_scratch();
    vector<int> soln;
    int soln_cost;
    bool isPlanFound = m_planner->replan(req.allocated_planning_time, 
                                         &soln, &soln_cost);

    if (isPlanFound){
        vector<FullBodyState> states =  m_env->reconstructPath(soln);
        vector<string> stat_names;
        vector<double> stats;
        packageStats(stat_names, stats, soln_cost, states.size());
        res.stats_field_names = stat_names;
        res.stats = stats;
        //for (auto state : states){
        //    printf("base states: ");
        //    for (auto value: state.base){
        //        printf("%f ", value);
        //    }
        //    printf("\n");
        //    printf("right arm angles: ");
        //    for (auto angle: state.right_arm){
        //        printf("%f ", angle);
        //    }
        //    printf("\n");
        //    printf("left arm angles: ");
        //    for (auto angle: state.left_arm){
        //        printf("%f ", angle);
        //    }
        //    printf("\n");
        //}
    } else {
        ROS_INFO("No plan found!");
    }
    return isPlanFound;
}

void EnvInterfaces::packageStats(vector<string>& stat_names, 
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size){
    
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "initial epsilon";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "final epsilon";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    // TODO fix the total planning time
    //stats[0] = totalPlanTime;
    stats[0] = m_planner->get_initial_eps_planning_time();
    stats[1] = m_planner->get_initial_eps_planning_time();
    stats[2] = m_planner->get_initial_eps();
    stats[3] = m_planner->get_n_expands_init_solution();
    stats[4] = m_planner->get_final_eps_planning_time();
    stats[5] = m_planner->get_final_epsilon();
    stats[6] = m_planner->get_solution_eps();
    stats[7] = m_planner->get_n_expands();
    stats[8] = static_cast<double>(solution_cost);
    stats[9] = static_cast<double>(solution_size);
}

bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name){
    m_collision_space_interface.bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic){
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::OccupancyGridPtr& map,
                              double new_origin_x, double new_origin_y,
                              double width, double height, 
                              vector<signed char>& final_map){
    vector<vector<signed char> > tmp_map(map->info.height);
    for (unsigned int i=0; i < map->info.height; i++){
        for (unsigned int j=0; j < map->info.width; j++){
            tmp_map[i].push_back(map->data.at(i*map->info.height+j));
        }
    }

    double res = map->info.resolution;
    int new_origin_x_idx = (new_origin_x-map->info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map->info.origin.position.y)/res;
    int new_width = width/res + 1;
    int new_height = height/res + 1;
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, width and height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    vector<vector<signed char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    final_map.resize(new_width * new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", final_map.size());
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            final_map[i*new_map[i].size() + j] = new_map[i][j];
        }
    }
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);
    // TODO look up the values for this cropping from the occup grid parameters
    vector<signed char> final_map;

    double width = 9;
    double height = 6;
    crop2DMap(map, 0, 0, width, height, final_map);
    nav_msgs::OccupancyGrid heur_map;
    heur_map.header.frame_id = "/map";
    heur_map.header.stamp = ros::Time::now();
    heur_map.info.map_load_time = ros::Time::now();
    heur_map.info.resolution = map->info.resolution;
    // done in the crop function too.
    heur_map.info.width = (width/map->info.resolution+1);
    heur_map.info.height = (height/map->info.resolution+1);
    heur_map.info.origin.position.x = 0;
    heur_map.info.origin.position.y = 0;
    heur_map.data = final_map;
    m_heur_map_pub.publish(heur_map);

    m_collision_space_interface.update2DHeuristicMaps(final_map);
}
