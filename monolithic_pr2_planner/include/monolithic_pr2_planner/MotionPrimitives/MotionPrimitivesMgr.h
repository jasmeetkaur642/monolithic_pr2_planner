#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseSnapMotionPrimitive.h>
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
            MotionPrimitivesMgr(GoalStatePtr& goal, std::vector<RobotState>&);
            bool loadMPrims(const MotionPrimitiveParams& files);
            void addIslandSnapPrimitives();
            void loadMPrimSet(int planning_mode);
            std::vector<MotionPrimitivePtr> getMotionPrims() { return m_active_mprims; };
            void searchNearGoal();
            void getUpdatedGoal(GoalStatePtr& goal) { m_goal = goal; }
            void getUpdatedGoalandTolerances(GoalStatePtr& goal, double xyz_tol, double roll_tol, double pitch_tol, double yaw_tol) {
                m_goal = goal;
                GoalStatePtr islandState;
                for(int i=0;i < m_islandStates.size();i++) {
                    islandState = boost::make_shared<GoalState>(m_islandStates[i], xyz_tol, roll_tol, pitch_tol, yaw_tol);
                    basesnap_mprim[i]->getUpdatedGoalandTolerances(islandState, xyz_tol, roll_tol, pitch_tol, yaw_tol);
                    basesnap_mprim[i]->m_end = goal;
                }

                armsnap_mprim->getUpdatedGoalandTolerances(m_goal, xyz_tol, roll_tol, pitch_tol, yaw_tol);

            }
            void updateParams(MotionPrimitiveParams);
        private:
            void loadBaseOnlyMPrims();
            void loadArmOnlyMPrims();
            void loadAllMPrims();
            void loadTorsoMPrims();
            void loadBaseSnapMPrims();
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
            GoalStatePtr m_goal;
            std::vector<BaseSnapMotionPrimitivePtr> basesnap_mprim;
            ArmSnapMotionPrimitivePtr armsnap_mprim;

            std::vector<RobotState> m_islandStates;
    };
}
