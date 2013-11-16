#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class BaseAdaptiveMotionPrimitive : public MotionPrimitive {
        public:
            BaseAdaptiveMotionPrimitive(int direction);
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor);
            virtual void print() const;

            //TODO this isn't the correct thing to do, but this is used as a
            //clue for the collision checker
            virtual int motion_type() const { return MPrim_Types::BASE; }; 
            virtual void computeCost(const MotionPrimitiveParams& params);
            static void goal(GoalState& goal) { m_goal = goal; };
            static GoalState goal() { return m_goal; };
        private:
            void computeIntermSteps(const GraphState& source_state,
                                    const GraphState& successor);
            int m_direction;
            // MotionPrimitiveParams m_params;
            // TODO yuck. shouldn't have this as a static variable
            static GoalState m_goal;
    };
    typedef boost::shared_ptr<BaseAdaptiveMotionPrimitive> BaseAdaptiveMotionPrimitivePtr;
}
