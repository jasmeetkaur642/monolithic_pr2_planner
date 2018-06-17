#include<ros/ros.h>
#include<monolithic_pr2_planner_node/GetMobileArmPlan.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner_node/fbp_stat_writer.h>
#include <sbpl/planners/mha_planner.h>

void printUsage(){
  printf("usage: runTests [imha | smha] [rr | ma | dts] path_to_problem start_goal_num\n");
  printf("usage: runTests [rrt | prm | rrtstar] path_to_problem\n");
}

bool isNumber(const std::string s) {
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

int main(int argc, char** argv){
  if(argc != 5 && argc != 4){
    printUsage();
    return 1;
  }
  ros::init(argc,argv,"run_problem");
  ros::NodeHandle nh;

  monolithic_pr2_planner_node::GetMobileArmPlan::Request req;
  monolithic_pr2_planner_node::GetMobileArmPlan::Response res;

  bool gotFilename = false;
  bool gotPlannerType = false;
  bool gotMetaType = false;
  req.use_ompl = false;
  char filename[100];
  std::string folder;
  // Defaults
  req.mha_type = mha_planner::MHAType::ORIGINAL;
  req.planner_type = mha_planner::PlannerType::SMHA;
  req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;

  for(int i=1; i<argc; i++){
    if(strcmp(argv[i],"imha")==0){
      req.planner_type = mha_planner::PlannerType::IMHA;
      gotPlannerType = true;
    }
    else if(strcmp(argv[i],"smha")==0){
      req.planner_type = mha_planner::PlannerType::SMHA;
      gotPlannerType = true;
    }
    else if(strcmp(argv[i],"rr")==0){
      req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
      req.mha_type = mha_planner::MHAType::ORIGINAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"rr_focal")==0){
      req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
      req.mha_type = mha_planner::MHAType::FOCAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"ma")==0){
      req.meta_search_type = mha_planner::MetaSearchType::META_A_STAR;
      req.mha_type = mha_planner::MHAType::ORIGINAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"ma_plus")==0){
      req.meta_search_type = mha_planner::MetaSearchType::META_A_STAR;
      req.mha_type = mha_planner::MHAType::PLUS;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"ma_focal")==0){
      req.meta_search_type = mha_planner::MetaSearchType::META_A_STAR;
      req.mha_type = mha_planner::MHAType::FOCAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"dts")==0){
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
      req.mha_type = mha_planner::MHAType::ORIGINAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"dts_original")==0){
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
      req.mha_type = mha_planner::MHAType::ORIGINAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"dts_plus")==0){
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
      req.mha_type = mha_planner::MHAType::PLUS;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"dts_focal")==0){
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
      req.mha_type = mha_planner::MHAType::FOCAL;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"dts_unconstrained")==0){
      req.meta_search_type = mha_planner::MetaSearchType::DTS;
      req.mha_type = mha_planner::MHAType::UNCONSTRAINED;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"gbfs")==0){
      req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
      req.mha_type = mha_planner::MHAType::GBFS;
      gotMetaType = true;
    }
    else if(strcmp(argv[i],"rrt")==0){
      req.use_ompl = true;
      gotMetaType = true;
    }
    else if(isNumber(argv[i])){
        std::cout<<argv[i]<<"\n";
        std::string relative_filename = "experiments/" + folder + "/start_goal" + argv[i] + ".yaml";
        //std::cout<<relative_filename<<"\n";
        strcpy(filename, relative_filename.c_str());
        std::cout<<filename<<"\n";
    }
    else{
      std::cout<<argv[i];
      folder = argv[i];
      std::string relative_filename = "experiments/" + folder + "/start_goal.yaml";
      //std::cout<<relative_filename<<"\n";
      strcpy(filename, relative_filename.c_str());
      gotFilename = true;
    }
  }

  if(!gotFilename || !( (gotPlannerType && gotMetaType) || req.use_ompl)){
    printUsage();
    return 1;
  }

  //planner parameters
  float eps = 50;
  if(nh.getParam("/monolithic_experiment/eps", eps))
      ROS_INFO("Got param inflation factor = %f", eps);
  else
      ROS_ERROR("Could not get inflation factor param");
  std::cout<<eps<<"\n";
  req.initial_eps = eps;
  req.final_eps = eps;
  req.dec_eps = 0.2;

  req.rarm_object.pose.position.x = 0;
  req.rarm_object.pose.position.y = 0;
  req.rarm_object.pose.position.z = 0;
  req.rarm_object.pose.orientation.x = 0;
  req.rarm_object.pose.orientation.y = 0;
  req.rarm_object.pose.orientation.z = 0;
  req.rarm_object.pose.orientation.w = 1;
  req.larm_object.pose.position.x = 0;
  req.larm_object.pose.position.y = 0;
  req.larm_object.pose.position.z = 0;
  req.larm_object.pose.orientation.x = 0;
  req.larm_object.pose.orientation.y = 0;
  req.larm_object.pose.orientation.z = 0;
  req.larm_object.pose.orientation.w = 1;

  req.xyz_tolerance = .04;
  req.roll_tolerance = .1;
  req.pitch_tolerance = .1;
  req.yaw_tolerance = .1;
  req.allocated_planning_time = 400;
  if (req.planner_type == mha_planner::PlannerType::IMHA)
      req.allocated_planning_time = req.allocated_planning_time *4;
  req.planning_mode = monolithic_pr2_planner::PlanningModes::RIGHT_ARM_MOBILE;

  req.body_start.resize(4);
  req.rarm_start.resize(7);
  req.larm_start.resize(7);
  req.body_goal.resize(4);
  req.rarm_goal.resize(7);
  req.larm_goal.resize(7);


  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<monolithic_pr2_planner_node::GetMobileArmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(filename,"r");
  if(!fin){
    printf("file %s does not exist\n", filename);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n    start:\n", &test_num) <= 0)
      break;

    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.start.pose.position.x,&req.start.pose.position.y,&req.start.pose.position.z,
              &req.start.pose.orientation.w,&req.start.pose.orientation.x,&req.start.pose.orientation.y,&req.start.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_start[0], &req.body_start[1],
              &req.body_start[2], &req.body_start[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_start[0],&req.rarm_start[1],
              &req.rarm_start[2],&req.rarm_start[3],
              &req.rarm_start[4],&req.rarm_start[5],
              &req.rarm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n    goal:\n",
              &req.larm_start[0],&req.larm_start[1],
              &req.larm_start[2],&req.larm_start[3],
              &req.larm_start[4],&req.larm_start[5],
              &req.larm_start[6]) <= 0)
      break;
    if(fscanf(fin,"      object_xyz_wxyz: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.goal.pose.position.x,&req.goal.pose.position.y,&req.goal.pose.position.z,
              &req.goal.pose.orientation.w,&req.goal.pose.orientation.x,&req.goal.pose.orientation.y,&req.goal.pose.orientation.z) <= 0)
      break;
    if(fscanf(fin,"      base_xyzyaw: %lf %lf %lf %lf\n",
              &req.body_goal[0], &req.body_goal[1],
              &req.body_goal[2], &req.body_goal[3]) <= 0)
      break;
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req.rarm_goal[0],&req.rarm_goal[1],
              &req.rarm_goal[2],&req.rarm_goal[3],
              &req.rarm_goal[4],&req.rarm_goal[5],
              &req.rarm_goal[6]) <= 0)
      break;
    if(fscanf(fin,"      larm: %lf %lf %lf %lf %lf %lf %lf\n\n",
              &req.larm_goal[0],&req.larm_goal[1],
              &req.larm_goal[2],&req.larm_goal[3],
              &req.larm_goal[4],&req.larm_goal[5],
              &req.larm_goal[6]) <= 0)
      break;

    printf("Running test %d\n",test_num);
    planner.call(req,res);
    FBPStatWriter::writeStatsToFile("mha_stats.csv", first, res.stats_field_names, res.stats);
    first = false;
  }

  return 0;
}

