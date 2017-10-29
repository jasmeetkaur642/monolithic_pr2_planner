#pragma once
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <monolithic_pr2_planner/PathPostProcessor.h>
#include <stdexcept>
#include <vector>
#include <memory>

#include <unordered_map>

#define NUM_SMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
#define NUM_IMHA_HEUR 4 // Used in EnvInterfaces to initialize the planner.
// This should include the Anchor search -> Total number of searches.

namespace monolithic_pr2_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */
    typedef std::pair<int, int> Edge;
    class Environment : public EnvironmentMHA {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            HeuristicMgrPtr getHeuristicMgr(){ return m_heur_mgr; };
            bool configureRequest(SearchRequestParamsPtr search_request_params,
                                  int& start_id, int& goal_id);
            void GetSuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);
            void GetSuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs);

            void GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);
            void GetLazySuccs(int q_id, int sourceStateID, vector<int>* succIDs, 
                vector<int>* costs, std::vector<bool>* isTrueCost);

            int GetTrueCost(int parentID, int childID);
            std::vector<FullBodyState> reconstructPath(std::vector<int> 
                state_ids);
            void reset();
            void setPlannerType(int planner_type);
            //void setUseNewHeuristics(bool use_new_heuristics){m_use_new_heuristics = use_new_heuristics;};
            void setHeuristicSetType(int heuristicSetType){m_heuristic_set_type = heuristicSetType;};
            
            void chooseSnapMprims();

        protected:
            bool setStartGoal(SearchRequestPtr search_request, 
                              int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureMotionPrimitives(SearchRequestPtr);
            void configureQuerySpecificParams(SearchRequestPtr search_request);
            void generateStartState(SearchRequestPtr search_request);

            ParameterCatalog m_param_catalog;
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
            ros::NodeHandle m_nodehandle;
            GoalStatePtr m_goal;
            RobotState m_start;
            bool m_using_lazy;
            MotionPrimitivesMgr m_mprims;
            HeuristicMgrPtr m_heur_mgr;

            int m_planner_type;
            int m_heuristic_set_type;

            bool m_visualizeIslands;

            //std::vector<RobotState> m_island_base_states;

        // SBPL interface stuff
        public:
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg){ return true; };
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ throw std::runtime_error("unimplement");  };
            int  GetGoalHeuristic(int stateID);
            int  GetGoalHeuristic(int heuristic_id, int stateID);
            int  GetStartHeuristic(int stateID) { throw std::runtime_error("unimplement"); };
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv(){ return m_hash_mgr->size(); };
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL){};
            void PrintEnv_Config(FILE* fOut){};

            void getIslandStates(std::vector<RobotState>&, std::vector<RobotState>&);

            void normalize_heuristic_times();
            void save_state_time(vector<int> soln_path);
            void save_heuristic_state_time(vector<int> soln_path);

            //Read island states and activation centers corresponding to each
            //start-goal pair from file.
            void readIslands();
            std::vector<int> countSnapMprimsApplied;
            std::vector<int> countSnapMprimsSucceeded;

            std::vector<int> countFbsMprimTried;
            std::vector<int> skipFbsMprimMod;

            std::vector<int> countBaseMprimTried;
            std::vector<int> skipBaseMprimMod;
            int m_numSnapMprims;

            std::map<Edge, MotionPrimitivePtr> m_edges;

            //Caches whether snapping is feasible from a graphstate to a snap island.
            std::set<std::pair<int, int> > m_infeasibleSnaps;

            std::unordered_map<int, std::pair<int, double> > m_state_time_map;
            std::vector<std::vector<std::pair<int, double> > > m_heuristic_state_time_map;
            //std::vector<std::vector<int> > m_heuristic_fbs_mprimid;
            //std::vector<std::vector<int> > m_heuristic_base_mprimid;

            std::vector<MotionPrimitivePtr> m_mprims_anchor;
            std::vector<MotionPrimitivePtr> m_mprims_inadmissible;

            // Contains the start goal pairs that have been run to generate
            // island states and activation centers. The indices of these three
            // variable correspond to each other.
            std::vector<std::pair<ContObjectState, ContObjectState> > m_startGoalPairs;
            std::vector<std::vector<RobotState> > m_islandStates;
            std::vector<std::vector<RobotState> > m_activationCenters;
    };
}
