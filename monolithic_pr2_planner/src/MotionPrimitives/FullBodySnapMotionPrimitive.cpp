#include <monolithic_pr2_planner/MotionPrimitives/FullBodySnapMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>
#include <angles/angles.h>
#include <algorithm>
#include <unistd.h>

using namespace monolithic_pr2_planner;
#define METER_TO_MM_MULT 1000

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
    ///ContBaseState cont_base_state = m_goal->getRobotState().getContBaseState();
    //DiscBaseState disc_base_state = m_goal->getRobotState().base_state();
    DiscBaseState baseActivationRadius = m_activationRadius.base_state();

    // XXX If a component in activationRadius is 0, it means that it is can be ignored, i.e, we can set a high threshold for it.
    // However, if the component is  non-zero, we need to respect it.
    int processedActivationRadiusX = 0, processedActivationRadiusY = 0;
    float processedActivationRadiusTheta = 0;
    if(baseActivationRadius.x() < d_tol.x())
        processedActivationRadiusX = 2*d_tol.x();
    else
        processedActivationRadiusX = baseActivationRadius.x();

    if(baseActivationRadius.y() < d_tol.y())
        processedActivationRadiusY = 2*d_tol.y();
    else
        processedActivationRadiusY = baseActivationRadius.y();
    
    if(baseActivationRadius.getContBaseState().theta() < c_tol.yaw())
        processedActivationRadiusTheta = 2*(c_tol.yaw());
    else
        processedActivationRadiusTheta = baseActivationRadius.getContBaseState().theta();

    setActivationRadiusInflation(2);

    int xDistance = abs(m_activationCenter.base_state().x() - base.x());
    int yDistance =  abs(m_activationCenter.base_state().y() - base.y());
    //ROS_ERROR("Distance to island= (%d, %d) and radius = (%d, %d)", xDistance, yDistance, m_inflationFactor*processedActivationRadiusX, m_inflationFactor*processedActivationRadiusY);

    bool within_activation_radius = (xDistance <
            m_inflationFactor*processedActivationRadiusX && yDistance <
            m_inflationFactor*processedActivationRadiusY);// &&
            //abs(angles::shortest_angular_distance(m_activationCenter.getContBaseState().theta(),
            //        robot_pose.getContBaseState().theta())) <
            //6*processedActivationRadiusTheta);
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
    //ROS_INFO("full bodysnap");
    //bool near_end = (abs(m_end->getRobotState().base_state().x()-base.x()) < 30*d_tol.x() &&
    //                          abs(m_end->getRobotState().base_state().y()-base.y()) < 30*d_tol.y());
    bool near_end = false;
    if(within_activation_radius && !near_end)
    { 
      RobotState rs = *m_goal;
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

    if(j_interpolate)
        ROS_INFO("fbs interpolated");
    for (auto robot_state: interp_steps){
        robot_state.printToDebug(MPRIM_LOG);
        //robot_state.visualize(100);
        //usleep(10000);
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
    m_cost = 10;
}
