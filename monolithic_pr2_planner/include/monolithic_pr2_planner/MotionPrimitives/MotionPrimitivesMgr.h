#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/FullBodySnapMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmSnapMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmTuckMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmUntuckMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/TorsoMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<MotionPrimitivePtr> MPrimList;
    class MotionPrimitivesMgr {
        public:
            MotionPrimitivesMgr(){};
            MotionPrimitivesMgr(GoalStatePtr&);
            MotionPrimitivesMgr(const GoalStatePtr& goal, const std::vector<RobotState>&, const std::vector<RobotState>&);
            bool loadMPrims(const MotionPrimitiveParams& files);
            void addIslandSnapPrimitives();
            void loadMPrimSet(int planning_mode);
            std::vector<MotionPrimitivePtr> getMotionPrims() { return m_active_mprims; };
            void searchNearGoal();
            void getUpdatedIslands(std::vector<RobotState> &islands, std::vector<RobotState> &activationCenters) {m_islandStates = islands, m_activationCenters = activationCenters;}
            void getUpdatedGoal(const GoalStatePtr& goal) { m_goal = goal; }
            void getUpdatedGoalandTolerances(const GoalStatePtr& goal, const
                    double xyz_tol, const double roll_tol, const double
                    pitch_tol, const double yaw_tol);
            void updateParams(MotionPrimitiveParams);
            void clearMprims() { 
                m_all_mprims = std::vector<std::vector<MotionPrimitivePtr> >(8);
                m_active_mprims.clear();
            }

            bool fullBodySnap;
            bool baseSnap;
            bool armSnap;
        private:
            void loadBaseOnlyMPrims();
            void loadArmOnlyMPrims();
            void loadAllMPrims();
            void loadTorsoMPrims();
            void loadBaseSnapMPrims();
            void loadFullBodySnapMPrims();
            void loadArmSnapMPrims();
            // these are all the possible mprims we have
            std::vector<std::vector<MotionPrimitivePtr> > m_all_mprims;
            void computeAllMPrimCosts(std::vector<MPrimList> mprims);
            double dist(DiscObjectState s1, DiscObjectState s2);
            MotionPrimitiveFileParser m_parser;

            // we're taking v1 and adding it into v2
            void combineVectors(const MPrimList& v1, MPrimList& v2);
            // these are the mprims that have been selected for use for this
            // planning request
            std::vector<MotionPrimitivePtr> m_active_mprims;
            MotionPrimitiveParams m_params;
            boost::shared_ptr<GoalState> m_goal;
            std::vector<BaseSnapMotionPrimitivePtr> basesnap_mprim;
            std::vector<FullBodySnapMotionPrimitivePtr> fullbody_snap_mprim;
            ArmSnapMotionPrimitivePtr armsnap_mprim;

            std::vector<RobotState> m_islandStates;
            std::vector<RobotState> m_activationCenters;
            // Caches failed graph id to graph id collision check.
            boost::shared_ptr<std::set<std::pair<int, int> > > m_infeasibleSnaps;
    };
}
