#include <monolithic_pr2_planner/MotionPrimitives/BaseSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

#define METER_TO_MM_MULT 1000
#define ARM_ANGLE_TOL 0.01

using namespace monolithic_pr2_planner;

bool BaseSnapMotionPrimitive::apply(const GraphState& source_state,
                           GraphStatePtr& successor,
                           TransitionData& t_data){
    //computeCost();

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

    DiscBaseState baseActivationRadius = m_activationRadius.base_state();
    DiscBaseState baseActivationCenter = m_activationCenter.base_state();

    std::vector<double> armActivationRadius, armActivationCenter, armSource;
    m_activationRadius.right_arm().getAngles(&armActivationRadius);
    m_activationCenter.right_arm().getAngles(&armActivationCenter);
    robot_pose.right_arm().getAngles(&armSource);

    //bool within_xyz_tol = (abs(m_goal->getObjectState().x()-obj.x()) < d_tol.x() &&
    //                       abs(m_goal->getObjectState().y()-obj.y()) < d_tol.y() &&
    //                       abs(m_goal->getObjectState().z()-obj.z()) < d_tol.z());


    bool within_basexy_tol = (abs(baseActivationCenter.x()-base.x()) < 25*baseActivationRadius.x() &&
                              abs(baseActivationCenter.y()-base.y()) < 25*baseActivationRadius.y());// &&
                              //abs(angles::shortest_angular_distance(m_activationCenter.getContBaseState().theta(), robot_pose.getContBaseState().theta())) < max(15*c_tol.yaw(), m_activationRadius.getContBaseState().theta()));

    ROS_ERROR("%f", m_end->getRobotState().getContBaseState().x());
    ROS_ERROR("1");
    bool near_end = (int)(abs(m_end->getRobotState().getContBaseState().x()/0.02) - base.x() < 40*d_tol.x() &&
                              (int)(abs(m_end->getRobotState().getContBaseState().y()/0.02)) - base.y()) < 40*d_tol.y();
    ROS_ERROR("2");

    bool within_arm_tol = true;
    //int i =0;
    //while(within_arm_tol && i < 7) {
    //    if(abs(angles::shortest_angular_distance(armSource[i], armActivationRadius[i])) > 10*armActivationRadius[i])
    //        within_arm_tol = false;
    //    i++;
    //}


      //ROS_ERROR("%f", m_goal->getRobotState().getContBaseState().x());
    if(within_basexy_tol && !near_end && within_arm_tol)
    {
      ROS_ERROR("%f", m_goal->getRobotState().getContBaseState().x());
      RobotState rs(m_goal->getRobotState().getContBaseState(), source_state.robot_pose().right_arm(), source_state.robot_pose().left_arm());
      successor.reset(new GraphState(rs));

      t_data.motion_type(motion_type());
      t_data.cost(cost());

     return computeIntermSteps(source_state, *successor, t_data);
    }
    else{
        return false;
    }
}

bool BaseSnapMotionPrimitive::computeIntermSteps(const GraphState& source_state,
                        const GraphState& successor,
                        TransitionData& t_data){

    ROS_DEBUG_NAMED(MPRIM_LOG, "interpolation for base snap primitive");

    ContBaseState start_base = source_state.robot_pose().getContBaseState();
    ContBaseState end_base = successor.robot_pose().getContBaseState();
    //vector<ContBaseState> interp_base_steps;
    int num_interp_steps = 0;

    num_interp_steps = numInterpSteps(source_state.robot_pose(), successor.robot_pose());

    m_interp_base_steps = ContBaseState::interpolate(start_base, end_base,
                                                     num_interp_steps);

    t_data.cont_base_interm_steps(m_interp_base_steps);

    std::vector<RobotState> interp_steps(num_interp_steps, source_state.robot_pose());
    t_data.interm_robot_steps(interp_steps);

    return true;
}

void BaseSnapMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG,
                    "BaseSnapMotionPrimitive cost %d", cost());
}

void BaseSnapMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    //TODO: Calculate actual cost
    m_cost = 6;
/*
    if(!m_interp_base_steps.size()) {
        ROS_INFO("default cost");
        m_cost = 3;
        m_params = params;
    }

    else {
        double linear_distance = 0;
        std::vector<ContBaseState> steps = m_interp_base_steps;

        ROS_ERROR("Size is %d", steps.size());
        for (size_t i=1; i < steps.size(); i++){
            double x0 = steps[i-1].x(); //[GraphStateElement::BASE_X];
            double y0 = steps[i-1].y(); //[GraphStateElement::BASE_Y];
            double x1 = steps[i].x(); //[GraphStateElement::BASE_X];
            double y1 = steps[i].y(); //[GraphStateElement::BASE_Y];
            double dx = x1-x0;
            double dy = y1-y0;
            linear_distance += sqrt(dx*dx + dy*dy);
        }
        std::cerr<<"vel "<<params.nominal_vel<<"\n";
        double linear_time = linear_distance/static_cast<double>(params.nominal_vel);
        double first_angle = steps[0].theta(); //[GraphStateElement::BASE_THETA];
        double final_angle = steps.back().theta(); //[GraphStateElement::BASE_THETA];
        double angular_distance = fabs(shortest_angular_distance(first_angle, 
                                                                final_angle));
        //assert((linear_time > 0) || (angular_distance > 0));

        double angular_time = angular_distance/params.angular_vel;

        //make the cost the max of the two times
        //m_cost = ceil(static_cast<double>(METER_TO_MM_MULT)*(max(linear_time, angular_time)));
        m_cost = ceil(static_cast<double>(max(linear_time, angular_time))); //Multiplying makes the cost too high.
        //use any additional cost multiplier
        //m_cost *= getAdditionalCostMult();
        ROS_INFO("base snap cost: %d", m_cost);

        assert(m_cost >= 0.0);
    }
    */
}

void BaseSnapMotionPrimitive::computeCost() {
    computeCost(m_params);
}

int BaseSnapMotionPrimitive::numInterpSteps(const RobotState& start, const RobotState& end){

    ContBaseState start_base = start.base_state();
    ContBaseState end_base = end.base_state();

    double dbase_theta = shortest_angular_distance(start_base.theta(),
                                                   end_base.theta());

    double d_rot = fabs(dbase_theta);

    double d_base = ContBaseState::distance(ContBaseState(start.base_state()),
                                         ContBaseState(end.base_state()));

    int rot_steps = static_cast<int>(d_rot/ContObjectState::getRPYResolution());
    int dist_steps = static_cast<int>(d_base/ContBaseState::getXYZResolution());

    int num_interp_steps = max(rot_steps, dist_steps);
    return num_interp_steps;
}
