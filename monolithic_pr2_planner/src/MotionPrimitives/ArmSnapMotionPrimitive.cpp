#include <monolithic_pr2_planner/MotionPrimitives/ArmSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;

bool ArmSnapMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){

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

    bool within_xyz_tol = (abs(m_goal->getObjectState().x()-base.x()) < 25*d_tol.x() &&
                           abs(m_goal->getObjectState().y()-base.y()) < 25*d_tol.y() &&
                           abs(m_goal->getObjectState().z()-base.z()) < 25*d_tol.z());


   // bool within_basexy_tol = (abs(m_goal->getRobotState().base_state().x()-base.x()) < 25*d_tol.x() &&
   //                           abs(m_goal->getRobotState().base_state().y()-base.y()) < 25*d_tol.y());

    /*
    bool ik_success = false;
    RobotState temp_pose(robot_pose.base_state(), robot_pose.right_arm(), robot_pose.left_arm());
    if(within_xyz_tol) {
        ROS_INFO("Inside arm tol");
        RobotPosePtr new_robot_pose_ptr;
        DiscObjectState disc_obj_state(m_goal->getRobotState().getObjectStateRelBody());

        ik_success = temp_pose.computeRobotPose(disc_obj_state, temp_pose, new_robot_pose_ptr);

        int c = 0;
        RightContArmState r_arm = temp_pose.right_arm();
        LeftContArmState l_arm = temp_pose.left_arm();
        while (!ik_success && c < 10000) {
            // ROS_ERROR("Failed to compute IK");
            r_arm.setUpperArmRoll(temp_pose.randomDouble(-3.75, 0.65));
            l_arm.setUpperArmRoll(temp_pose.randomDouble(-3.75, 0.65));
            temp_pose.right_arm(r_arm);
            temp_pose.left_arm(l_arm);
            ik_success = temp_pose.computeRobotPose(disc_obj_state, temp_pose, new_robot_pose_ptr);
            c++;
        }
    }

    */
    if(within_xyz_tol)// && ik_success)
    { 
      RobotState source_pose = source_state.robot_pose();

      ContObjectState goal_rel_map = m_goal->getRobotState().getObjectStateRelMap();
      ContObjectState goal_rel_body = m_goal->getRobotState().getObjectStateRelBody();

      ROS_ERROR("Goal rel orig body");
      ROS_INFO("%f\t%f\t%f", goal_rel_body.x(), goal_rel_body.y(), goal_rel_body.z());

      KDL::Rotation rot = KDL::Rotation::RPY(goal_rel_map.roll(), goal_rel_map.pitch(), goal_rel_map.yaw());
      KDL::Vector vec(goal_rel_map.x(), goal_rel_map.y(), goal_rel_map.z());
      KDL::Frame wrt_map(rot, vec);

      ContBaseState cont_base_state = source_pose.getContBaseState();

      rot = KDL::Rotation::RPY(0, 0, cont_base_state.theta());
      //vec = KDL::Vector(source_pose.base_state().x()*0.02 , source_pose.base_state().y()*0.02, 0.803 + source_pose.base_state().z()*0.02); //Is the Z values correct?
      vec = KDL::Vector(cont_base_state.x() + 0.0501, +cont_base_state.y(), cont_base_state.z() + 0.803);
      KDL::Frame source_frame(rot, vec);
      ROS_INFO("Yaw: %f", cont_base_state.theta());
      //ROS_ERROR("Robot base frame");
      //ROS_INFO("%f\t%f\t%f", source_frame.p.x(), source_frame.p.y(), source_frame.p.z());

      //ROS_ERROR("Wrt map");
      //ROS_INFO("%f\t%f\t%f", wrt_map.p.x(), wrt_map.p.y(), wrt_map.p.z());

      KDL::Frame wrt_body = source_frame.Inverse() * wrt_map;
      //vec = KDL::Vector(wrt_body.p.x() + 0.051, wrt_body.p.y(), wrt_body.p.z());
      //wrt_body = KDL::Frame(wrt_body.M, vec);
      ROS_ERROR("Goal Wrt body");
      ROS_INFO("%f\t%f\t%f", wrt_body.p.x(), wrt_body.p.y(), wrt_body.p.z());

      wrt_map = source_frame * wrt_body;
      ROS_INFO("%f\t%f\t%f", wrt_map.p.x(), wrt_map.p.y(), wrt_map.p.z());

      //RobotState rs(source_state.robot_pose().getContBaseState(), m_goal->getRobotState().right_arm(), m_goal->getRobotState().left_arm());
      double wr, wp, wy;
      wrt_body.M.GetRPY(wr, wp, wy);
      ContObjectState goal_obj_wrt_body(wrt_body.p.x(), wrt_body.p.y(), wrt_body.p.z(), wr, wp, wy);


      RobotPosePtr new_robot_pose_ptr;
      bool ik_success = source_pose.computeRobotPose(goal_obj_wrt_body, source_pose, new_robot_pose_ptr);
      RobotState temp(source_pose.getContBaseState(), source_pose.right_arm(), source_pose.left_arm());
      RightContArmState right_arm = source_pose.right_arm();
      LeftContArmState left_arm = source_pose.left_arm();

        int c = 0;
        while (!ik_success && c < 10000) {
            // ROS_ERROR("Failed to compute IK");
            right_arm.setUpperArmRoll(temp.randomDouble(-3.75, 0.65));
            //left_arm.setUpperArmRoll(temp.randomDouble(-3.75, 0.65));
            temp.right_arm(right_arm);
            //temp.left_arm(left_arm);
            ik_success = temp.computeRobotPose(goal_obj_wrt_body, temp, new_robot_pose_ptr);
            c++;
        }
      if(ik_success) {
        ROS_INFO("Snapping to goal state");
        //RobotState rs(source_pose.getContBaseState(), goal_obj_wrt_body);
        RobotState rs(source_pose.getContBaseState(), right_arm, left_arm);
        //rs.visualize(200);
        //sleep(2);
        successor.reset(new GraphState(rs));

        t_data.motion_type(motion_type());
        t_data.cost(cost());
        return computeIntermSteps(source_state, *successor, t_data);
      }
      else {
          ROS_INFO("IK failed");
          return false;
      }
    
    }
    else{
        return false;
    } 
}

bool ArmSnapMotionPrimitive::computeIntermSteps(const GraphState& source_state, 
                        const GraphState& successor, 
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for arm snap primitive");
    std::vector<RobotState> interp_steps;
    bool interpolate = RobotState::workspaceInterpolate(source_state.robot_pose(), 
                                     successor.robot_pose(),
                                     &interp_steps);
    bool j_interpolate;

    if (!interpolate) {
        interp_steps.clear();
        j_interpolate = RobotState::jointSpaceInterpolate(source_state.robot_pose(),
                                    successor.robot_pose(), &interp_steps);
    }

    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
    }
    t_data.interm_robot_steps(interp_steps);

    if(!interpolate && !j_interpolate)
    {
        ROS_WARN("No valid arm interpolation found to snap arm to goal pose");
        return false;
    }

    ContBaseState c_base = source_state.robot_pose().getContBaseState();
    std::vector<ContBaseState> cont_base_states(interp_steps.size(), c_base);
    t_data.cont_base_interm_steps(cont_base_states);

    return true;
}

void ArmSnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, 
                    "ArmSnapMotionPrimitive cost %d", cost());
}

void ArmSnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost 
    m_cost = 1;
}
