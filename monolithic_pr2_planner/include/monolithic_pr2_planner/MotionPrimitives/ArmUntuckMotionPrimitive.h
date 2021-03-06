#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
  class ArmUntuckMotionPrimitive : public MotionPrimitive {
    public:
      ArmUntuckMotionPrimitive(bool full){full_untuck_ = full;};
      virtual bool apply(const GraphState& graph_state, 
          GraphStatePtr& successor,
          TransitionData& t_data);
      virtual void print() const;
      virtual int motion_type() const { return MPrim_Types::ARM_ADAPTIVE; }; 
      virtual void computeCost(const MotionPrimitiveParams& params);
      void computeIntermSteps(const GraphState& source_state, 
                                    const GraphState& successor, 
                                    TransitionData& t_data);
    private:
      bool full_untuck_;
  };
  typedef boost::shared_ptr<ArmUntuckMotionPrimitive> ArmUntuckMotionPrimitivePtr;

}
