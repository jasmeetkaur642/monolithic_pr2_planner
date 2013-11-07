#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/Constants.h>
#include <assert.h>

namespace monolithic_pr2_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    class MotionPrimitive {
        public:
            MotionPrimitive();
            void setID(int id) { m_id = id; };
            int getID() const { return m_id; };
            virtual void setIntermSteps(IntermSteps& coord) { m_interm_steps = coord; };
            virtual IntermSteps getIntermSteps(){ return m_interm_steps; };
            virtual void setEndCoord(GraphStateMotion& coord); 
            virtual bool apply(const GraphState& graph_state, 
                               std::unique_ptr<GraphState>& successor) = 0;
            virtual void print() const = 0;
            virtual int getMotionType() const = 0;
            virtual void computeCost(const MotionPrimitiveParams& params) = 0;
            virtual void printIntermSteps() const;
            virtual void printEndCoord() const;
            virtual int getCost() const { return m_cost; };
            virtual void setAdditionalCostMult(double cost) { m_additional_cost = cost; };
            virtual int getAdditionalCostMult() { return m_additional_cost; };

        protected:
            int m_id;
            int m_cost;
            int m_additional_cost;
            GraphStateMotion m_end_coord;
            IntermSteps m_interm_steps;

    };
    typedef boost::shared_ptr<MotionPrimitive> MotionPrimitivePtr;
}
