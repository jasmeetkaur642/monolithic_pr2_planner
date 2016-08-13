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
    BaseSnapMotionPrimitive(RobotState& goal): m_goal_robot(goal), m_tolerances(4,0) {}
    virtual bool apply(const GraphState& graph_state,
        GraphStatePtr& successor,
        TransitionData& t_data);
    virtual void print() const;
    virtual int motion_type() const { return MPrim_Types::BASE_SNAP; };
    virtual void computeCost(const MotionPrimitiveParams& params);
    int numInterpSteps(const RobotState& start, const RobotState& end);
    bool computeIntermSteps(const GraphState& source_state,
                        const GraphState& successor,
                        TransitionData& t_data);
    void getUpdatedGoalandTolerances(RobotState& goal,double xyz_tol, double roll_tol, double pitch_tol, double yaw_tol)
    {
        m_goal = boost::make_shared<GoalState>(m_goal_robot, xyz_tol, roll_tol, pitch_tol, yaw_tol);
        m_tolerances[Tolerances::XYZ] =  xyz_tol;
        m_tolerances[Tolerances::ROLL] =  roll_tol;
        m_tolerances[Tolerances::PITCH] =  pitch_tol;
        m_tolerances[Tolerances::YAW] =  yaw_tol;
    }

    GoalStatePtr m_goal;
    RobotState m_goal_robot;
    std::vector<double> m_tolerances;
};
typedef boost::shared_ptr<BaseSnapMotionPrimitive> BaseSnapMotionPrimitivePtr;

}
