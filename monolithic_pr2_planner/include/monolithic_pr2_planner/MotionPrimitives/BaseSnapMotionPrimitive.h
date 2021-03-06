#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
class BaseSnapMotionPrimitive : public MotionPrimitive {
    public:
    BaseSnapMotionPrimitive():m_tolerances(4,0)
    {
    //Quickly hardcoded. Should be read from SearchReqParam

    }
    BaseSnapMotionPrimitive(RobotState& goal): m_tolerances(4,0) {m_goal = boost::make_shared<RobotState>(goal);}
    virtual bool apply(const GraphState& graph_state,
        GraphStatePtr& successor,
        TransitionData& t_data);
    virtual void print() const;
    virtual int motion_type() const { return MPrim_Types::BASE_SNAP; };
    void computeCost();
    virtual void computeCost(const MotionPrimitiveParams& params);
    int numInterpSteps(const RobotState& start, const RobotState& end);
    bool computeIntermSteps(const GraphState& source_state,
                        const GraphState& successor,
                        TransitionData& t_data);
    // Update the goal and tolerances.
    // Called primarily by the MotionPrimitiveMgr.
    void getUpdatedGoalandTolerances(const RobotPosePtr& goal, const  double xyz_tol, const  double roll_tol, const double pitch_tol, const double yaw_tol)
    {
        m_goal = goal;
        m_tolerances[Tolerances::XYZ] =  xyz_tol;
        m_tolerances[Tolerances::ROLL] =  roll_tol;
        m_tolerances[Tolerances::PITCH] =  pitch_tol;
        m_tolerances[Tolerances::YAW] =  yaw_tol;
    }
    void updateParams(MotionPrimitiveParams params) {m_params = params;}

    RobotPosePtr m_goal;
    GoalStatePtr m_end; // m_end is actually the of search. We want to deactivate base snap near goal.
    MotionPrimitiveParams m_params;
    std::vector<double> m_tolerances;
    std::vector<ContBaseState> m_interp_base_steps;

    RobotState m_activationCenter;
    RobotState m_activationRadius;
};
typedef boost::shared_ptr<BaseSnapMotionPrimitive> BaseSnapMotionPrimitivePtr;

}
