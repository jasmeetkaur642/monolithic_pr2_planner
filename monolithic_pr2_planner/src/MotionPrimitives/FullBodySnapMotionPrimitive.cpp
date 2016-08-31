#include <monolithic_pr2_planner/MotionPrimitives/FullBodySnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>
#include <angles/angles.h>
#include <algorithm>

using namespace monolithic_pr2_planner;

boost::shared_ptr<std::set<std::pair<int, int> > > FullBodySnapMotionPrimitive::m_infeasibleSnaps;

bool FullBodySnapMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

    //if(m_infeasibleSnaps->count(std::make_pair(source_state.id(), GraphState(m_goal->getRobotState()).id()))) {
    //    ROS_INFO("Snap not feasible");
    //    return false;
    //}

    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    
    RobotState robot_pose = source_state.robot_pose();
    DiscObjectState obj = source_state.getObjectStateRelMap();
    DiscBaseState base = robot_pose.base_state();
    unsigned int r_free_angle = robot_pose.right_free_angle();
    ContBaseState cont_base_state = m_goal->getRobotState().getContBaseState();
    DiscBaseState disc_base_state = m_goal->getRobotState().base_state();
    DiscBaseState baseActivationRadius = m_activationRadius.base_state();

//    bool within_xyz_tol = (abs(m_goal->getObjectState().x()-obj.x()) < d_tol.x() &&
//                           abs(m_goal->getObjectState().y()-obj.y()) < d_tol.y() &&
//                           abs(m_goal->getObjectState().z()-obj.z()) < d_tol.z());
//

  //  bool within_basexy_tol = (abs(m_goal->getRobotState().base_state().x()-base.x()) < 3*d_tol.x() &&
  //                            abs(m_goal->getRobotState().base_state().y()-base.y()) < 3*d_tol.y() &&
  //                            abs(angles::shortest_angular_distance(cont_base_state.theta(), robot_pose.getContBaseState().theta())) < 15*c_tol.yaw());

    //ROS_INFO("%f, %f", abs(angles::shortest_angular_distance(cont_base_state.theta(), robot_pose.getContBaseState().theta())), abs(angles::shortest_angular_distance(m_activationCenter.getContBaseState().theta(), cont_base_state.theta())));
    //ROS_INFO("%d, %d", abs(disc_base_state.x() - base.x()), abs(m_activationCenter.base_state().x() - disc_base_state.x()));
    bool within_activation_radius = (abs(m_activationCenter.base_state().x() - base.x()) < max(4*d_tol.x(), baseActivationRadius.x()) &&
                                    abs(m_activationCenter.base_state().y() - base.y()) < max(4*d_tol.y(), baseActivationRadius.y()) &&
                                    abs(angles::shortest_angular_distance(m_activationCenter.getContBaseState().theta(), robot_pose.getContBaseState().theta())) < max(4*c_tol.yaw(), m_activationRadius.getContBaseState().theta()));
    /*
    bool temp;
    std::vector<double> r_arm_goal, r_arm_center, r_arm_source;
    m_goal->getRobotState().right_arm().getAngles(&r_arm_goal);
    m_activationCenter.right_arm().getAngles(&r_arm_center);
    robot_pose.right_arm().getAngles(&r_arm_source);

    if(within_activation_radius)
        for(int i=0;i<7;i++) {
            ROS_INFO("%f", abs(angles::shortest_angular_distance(r_arm_center[i], r_arm_source[i])));
            if(abs(angles::shortest_angular_distance(r_arm_source[i], r_arm_goal[i])) > abs(angles::shortest_angular_distance(r_arm_center[i], r_arm_source[i]))) {
                within_activation_radius = false;
                break;
            }
        }
        */
   // bool near_end = (abs(m_end->getRobotState().base_state().x()-base.x()) < 30*d_tol.x() &&
   //                           abs(m_end->getRobotState().base_state().y()-base.y()) < 30*d_tol.y());
   bool near_end = false;
    if(within_activation_radius && !near_end)
    { 
      //ROS_INFO("[FBS] Search near goal");      

      RobotState rs = m_goal->getRobotState();
      successor.reset(new GraphState(rs));
      //sleep(2);

      t_data.motion_type(motion_type());
      t_data.cost(cost());
    
     return computeIntermSteps(source_state, *successor, t_data);
    }
    else{
        return false;
    } 
}

bool FullBodySnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for full body snap primitive");
    std::vector<RobotState> interp_steps;
    //bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
    //                                 successor.robot_pose(),
    //                                 &interp_steps);
    bool interpolate = false;
    bool j_interpolate;
    j_interpolate = RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                    successor.robot_pose(), &interp_steps);

    //if (!interpolate) {
    //    interp_steps.clear();
    //    j_interpolate = RobotState::jointSpaceInterpolate(source_state.robot_pose(),
    //                                successor.robot_pose(), &interp_steps);
    //}

    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);

    if(!interpolate && !j_interpolate)
    {
        ROS_WARN("No valid arm interpolation found to snap to full body goal pose");
        return false;
    }

    std::vector<ContBaseState> cont_base_states(interp_steps.size());
    for(size_t i = 0; i < interp_steps.size(); i++)
      cont_base_states[i] = interp_steps[i].getContBaseState();

    t_data.cont_base_interm_steps(cont_base_states);

    return true;
}

void FullBodySnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "FullBodySnapMotionPrimitive cost %d", cost());
}

void FullBodySnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 50;
}
