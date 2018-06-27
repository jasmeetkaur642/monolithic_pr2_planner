#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <cassert>
#include <iomanip>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh)
    :   m_hash_mgr(new HashManager(&StateID2IndexMapping)),
        m_nodehandle(nh), //m_mprims(m_goal),
        m_heur_mgr(new HeuristicMgr()),
        m_using_lazy(false),
        m_planner_type(T_SMHA) {
        m_goal = make_shared<GoalState>();
        //m_mprims = MotionPrimitivesMgr(m_goal);
        //std::vector<RobotState> islandStates = getBaseIslandStates();
        //m_mprims = MotionPrimitivesMgr(m_goal, islandStates);
        m_param_catalog.fetch(nh);
        configurePlanningDomain();
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void Environment::reset() {
    m_heur_mgr->reset();
    // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));
    m_edges.clear();

    // Fetch params again, in case they're being modified between calls.
    // m_param_catalog.fetch(m_nodehandle);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
void Environment::setPlannerType(int planner_type) {
    m_planner_type = planner_type;
    m_heur_mgr->setPlannerType(planner_type);
    ROS_INFO_NAMED(SEARCH_LOG, "Setting planner type: %d", m_planner_type);
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id) {
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(
        search_request_params));
    configureQuerySpecificParams(search_request);

    if(search_request->m_params->underspecified_start) {
        ROS_DEBUG_NAMED(CONFIG_LOG, "underspecified_start. Will generate start state.");
        generateStartState(search_request);
    }
    if (!setStartGoal(search_request, start_id, goal_id)) {
        return false;
    }
    // This needs the goal state to be set for choosing islands.
    configureMotionPrimitives(search_request);

    return true;
}

int Environment::GetGoalHeuristic(int stateID) {
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(0, stateID);
}

int Environment::GetGoalHeuristic(int heuristic_id, int stateID) {
    // Update params in snap primitives.
    //m_param_catalog.fetch(m_nodehandle);
    //m_mprims.updateParams(m_param_catalog.m_motion_primitive_params);

    GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);
    if(m_goal->isSatisfiedBy(successor) || stateID == GOAL_STATE){
        return 0;
    }

    std::unique_ptr<stringintmap> values;
    m_heur_mgr->getGoalHeuristic(successor, values);

    for (auto& heur : (*values)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "%s : %d", heur.first.c_str(), heur.second);
    }

    //XXX This is VERY brittle.
    //If you add another set type, make sure to set the appropriate number of
    //planner_queues in Envinterfaces.
    if(m_heuristic_set_type == 0){
        switch (heuristic_id) {
            case 0:  // Anchor
            return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
            case 1:  // ARA Heur
            return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
            case 2:  // Base1, Base2 heur
            return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
            case 3:
            return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_vert"));
        }
    }
    else if(m_heuristic_set_type == 1){
        double w_bfsRot = 0.2;
        double w_armFold = 0.2;
        int ad_base = (*values).at("admissible_base");
        int ad_endeff = (*values).at("admissible_endeff");
        int anchor_h = std::max(ad_base, ad_endeff);
        int endeff_rot_goal = (*values).at("endeff_rot_goal");

        int inad_arm_heur = static_cast<int>(0.1*(*values).at("endeff_rot_goal") + 0.1*ad_endeff);
        if (ad_base > 1000) {  //TODO: check multiplier
            inad_arm_heur = (*values).at("arm_angles_folded");
        }
        switch (heuristic_id) {
        case 0:  // Anchor
            return anchor_h;
        case 1:  // Anchor
            return int(0.1*ad_base) + int(0.1*ad_endeff) + int(0.2*endeff_rot_goal);
            //return anchor_h;
        case 2:  // Base1, Base2 heur
            return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("endeff_rot_goal"));
        case 3:  // Base1, Base2 heur
            //return static_cast<int>(1.0*(*values).at("base_with_rot_0") + 0.0*(*values).at("endeff_rot_goal"));
            return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("arm_angles_folded"));
        case 4:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*inad_arm_heur);
        case 5:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot1") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*inad_arm_heur);
        case 6:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot8") + w_armFold*inad_arm_heur);
        case 7:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot3") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot11") + w_armFold*inad_arm_heur);
        case 8:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot4") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot14") + w_armFold*inad_arm_heur);
        case 9:
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*inad_arm_heur);
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*inad_arm_heur);
        }
    }
    else if(m_heuristic_set_type == 2) {
        double w_bfsRot = 0.2;
        double w_armFold = 0.2;
        int ad_base = (*values).at("admissible_base");
        int ad_endeff = (*values).at("admissible_endeff");
        int anchor_h = std::max(ad_base, ad_endeff);
        int endeff_rot_goal = (*values).at("endeff_rot_goal");

        int inad_arm_heur = static_cast<int>(0.1*(*values).at("endeff_rot_goal") + 0.1*ad_endeff);
        switch (heuristic_id) {
            case 0:  // Anchor
            return anchor_h;
            case 1:  // Anchor
            return int(0.1*ad_base) + int(0.1*ad_endeff) + int(0.2*endeff_rot_goal);
            //return ad_base;
            case 2:  // Base1, Base2 heur
            return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("endeff_rot_goal"));
            case 3:  // Base1, Base2 heur
            //return static_cast<int>(1.0*(*values).at("base_with_rot_0") + 0.0*(*values).at("endeff_rot_goal"));
            return static_cast<int>(0.1*(*values).at("base_with_rot_0") + 0.1*(*values).at("arm_angles_folded"));
            //return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*inad_arm_heur);
            case 4:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*inad_arm_heur);
            case 5:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot1") + w_armFold*inad_arm_heur);
            case 6:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*inad_arm_heur);
            case 7:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot3") + w_armFold*inad_arm_heur);
            case 8:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot4") + w_armFold*inad_arm_heur);
            case 9:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*inad_arm_heur);
            case 10:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot6") + w_armFold*inad_arm_heur);
            case 11:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot7") + w_armFold*inad_arm_heur);
            case 12:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot8") + w_armFold*inad_arm_heur);
            case 13:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot9") + w_armFold*inad_arm_heur);
            case 14:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot10") + w_armFold*inad_arm_heur);
            case 15:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot11") + w_armFold*inad_arm_heur);
            case 16:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot12") + w_armFold*inad_arm_heur);
            case 17:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot13") + w_armFold*inad_arm_heur);
            case 18:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot14") + w_armFold*inad_arm_heur);
            case 19:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*inad_arm_heur);
        }
    }
    else if(m_heuristic_set_type == 3){
        double w_bfsRot = 0.2;
        double w_armFold = 0.2;
        int ad_base = (*values).at("admissible_base");
        int ad_endeff = (*values).at("admissible_endeff");
        int endeff_rot_goal = (*values).at("endeff_rot_goal");

        int inad_arm_heur = static_cast<int>(0.1*(*values).at("endeff_rot_goal") + 0.1*ad_endeff);
      switch (heuristic_id) {
        case 0:  // Anchor
            return (*values).at("admissible_base");
        case 1:  // ARA Heur
            return inad_arm_heur;
        case 2:
            return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*inad_arm_heur);
        //case 2:  // Base1, Base2 heur
          //return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
        //case 3:
          //return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_vert"));
      }
    }
    else
        ROS_ERROR("Incorrect heuristic set type supplied.");

    return std::max((*values).at("admissible_base"), (*values).at("admissible_endeff"));
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs,
                           vector<int>* costs){
    GetSuccs(0, sourceStateID, succIDs, costs);
}

void deleteMPrim(std::vector<MotionPrimitivePtr> &active_mprims, int id, int type) {
    //Assumes that id is the index of the mprim in m_all_mprims.
    for(int i=0;i<active_mprims.size();i++) {
        auto mprim = active_mprims[i];
        if(mprim->motion_type() == type && mprim->getID() == id) {
            active_mprims.erase(std::remove(active_mprims.begin(), active_mprims.end(), mprim), active_mprims.end());
            break;
        }
    }
}

void Environment::GetSuccs(int q_id, int sourceStateID, vector<int>* succIDs,
                           vector<int>* costs){
    assert(sourceStateID != GOAL_STATE);

    //Code to calculate node expansion time.
    // insert needs to be used as pathPostProcessor also calls getSuccs once
    // path is found.
    m_state_time_map.insert(std::make_pair(sourceStateID, std::make_pair(q_id, clock() / (double)CLOCKS_PER_SEC)));

    //m_heuristic_state_time_map[q_id].push_back(std::make_pair(sourceStateID, (double)(clock()) / CLOCKS_PER_SEC));


    ROS_DEBUG_NAMED(SEARCH_LOG,
            "==================Expanding state %d==================",
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if (m_param_catalog.m_visualization_params.expansions) {
        RobotState expansion_pose = source_state->robot_pose();
        expansion_pose.visualize(250/NUM_SMHA_HEUR*q_id);
        // source_state->robot_pose().visualize(250/NUM_SMHA_HEUR*q_id);
        m_cspace_mgr->visualizeAttachedObject(expansion_pose, 250/NUM_SMHA_HEUR*q_id);
        // m_cspace_mgr->visualizeCollisionModel(expansion_pose);
        //usleep(5000);
    }
    //srand(time(NULL));
    //std::vector<int> mPrimID(7);
    //for(int m=0;m<5;m++){
    //    mPrimID.push_back( rand() % m_mprims.getMotionPrims().size());
    //}

    //std::vector<std::pair<int, int> > fbsDeleteMprims;
    //std::vector<std::pair<int, int> > baseDeleteMprims;
    std::vector<std::pair<int, int> > deleteMprims;

    std::vector<MotionPrimitivePtr> active_mprims;
    if(q_id == 0)
        active_mprims = m_mprims_anchor;
    else
        active_mprims = m_mprims_inadmissible;
    //for (auto mprim : m_mprims.getMotionPrims()) {
    for(auto mprim : active_mprims) {
        //Keep separate list of snap mprims for each heuristic so that we don't
        //keep collision checking once an activation center has been expanded.

        //if(mprim->motion_type() == MPrim_Types::FULLBODY_SNAP &&
        //        std::find(m_heuristic_fbs_mprimid[q_id].begin(),
        //            m_heuristic_fbs_mprimid[q_id].end(), mprim->getID()) ==
        //        m_heuristic_fbs_mprimid[q_id].end())
        //    continue;
        //if(mprim->motion_type() == MPrim_Types::BASE_SNAP &&
        //        std::find(m_heuristic_base_mprimid[q_id].begin(),
        //            m_heuristic_base_mprimid[q_id].end(), mprim->getID()) ==
        //        m_heuristic_base_mprimid[q_id].end())
        //    continue;


        ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        // mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;


        //if((mprim->motion_type() == MPrim_Types::BASE_SNAP) && q_id != 0)
        //    continue;

        if (!mprim->apply(*source_state, successor, t_data)) {
            ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
            continue;
        }
        if(mprim->motion_type() == MPrim_Types::BASE_SNAP) {
            //countBaseMprimTried[mprim->getID()] ++;
            //if(countBaseMprimTried[mprim->getID()] % skipBaseMprimMod[mprim->getID()])
            //    continue;
            //ROS_INFO("Appling fbs mprim tried: %d, mod: %d", countBaseMprimTried[mprim->getID()], skipBaseMprimMod[mprim->getID()]);
            countSnapMprimsApplied[0] ++;
        }
        else if (mprim->motion_type() == MPrim_Types::FULLBODY_SNAP) {
            //countFbsMprimTried[mprim->getID()] ++;
            //if(countFbsMprimTried[mprim->getID()] % skipFbsMprimMod[mprim->getID()])
            //    continue;
            //ROS_INFO("Appling fbs mprim tried: %d, mod: %d", countFbsMprimTried[mprim->getID()], skipFbsMprimMod[mprim->getID()]);
            countSnapMprimsApplied[1] ++;
        }

        if (m_cspace_mgr->isValidSuccessor(*successor,t_data) &&
            m_cspace_mgr->isValidTransitionStates(t_data)){

            ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
            source_state->printToDebug(SEARCH_LOG);
            m_hash_mgr->save(successor);
            ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:",
                            successor->id());
            successor->printToDebug(MPRIM_LOG);

            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
                    mprim->cost());
                succIDs->push_back(GOAL_STATE);
            } else {
                succIDs->push_back(successor->id());
            }
            costs->push_back(mprim->cost());
            ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());

            if(t_data.motion_type() == MPrim_Types::BASE_SNAP) {
                ROS_ERROR("Queue: %d; Base snap motion succeeded", q_id);
                countSnapMprimsSucceeded[0]++;
                deleteMprims.push_back(std::make_pair(mprim->getID(), mprim->motion_type()));
                //m_heuristic_base_mprimid[q_id].erase(std::remove(m_heuristic_base_mprimid[q_id].begin(), m_heuristic_base_mprimid[q_id].end(), mprim->getID()), m_heuristic_base_mprimid[q_id].end());
            }
            if(mprim->motion_type() == MPrim_Types::FULLBODY_SNAP) {
                ROS_ERROR("Queue: %d, FBS succeeded, state-id is %d", q_id, successor->id());
                countSnapMprimsSucceeded[1]++;
                deleteMprims.push_back(std::make_pair(mprim->getID(), mprim->motion_type()));
                //m_heuristic_fbs_mprimid[q_id].erase(std::remove(m_heuristic_fbs_mprimid[q_id].begin(), m_heuristic_fbs_mprimid[q_id].end(), mprim->getID()), m_heuristic_fbs_mprimid[q_id].end());
            }
            if(mprim->motion_type() == MPrim_Types::ARM_SNAP) {
                ROS_ERROR("Queue: %d, Arm SNAP succeeded", q_id);
            }
        } else {
            //if(mprim->motion_type() == MPrim_Types::FULLBODY_SNAP) {
            //    countFbsMprimTried[mprim->getID()] = 0;
            //    skipFbsMprimMod[mprim->getID()] *= 2;
            //}
            //if(mprim->motion_type() == MPrim_Types::BASE_SNAP) {
            //    countBaseMprimTried[mprim->getID()] = 0;
            //    skipBaseMprimMod[mprim->getID()] *= 2;
            //}
            ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
        }
    }
    //for(auto idPair : fbsDeleteMprims){
    //    m_mprims.deleteMPrim(idPair.first, idPair.second);
    //    ROS_ERROR("Mprim id= %d, type= %d deleted", idPair.first, idPair.second);
    //}
    //for(auto idPair : baseDeleteMprims){
    //    m_mprims.deleteMPrim(idPair.first, idPair.second);
    //    ROS_ERROR("Mprim id= %d, type= %d deleted", idPair.first, idPair.second);
    //}

    if(q_id == 0) {
        for(auto idPair : deleteMprims){
            deleteMPrim(m_mprims_anchor, idPair.first, idPair.second);
            ROS_ERROR("Mprim id= %d, type= %d deleted", idPair.first, idPair.second);
        }
    }
    else {
        for(auto idPair : deleteMprims){
            deleteMPrim(m_mprims_inadmissible, idPair.first, idPair.second);
            ROS_ERROR("Mprim id= %d, type= %d deleted", idPair.first, idPair.second);
        }
    }
}

void Environment::GetLazySuccs(int sourceStateID, vector<int>* succIDs,
                           vector<int>* costs, std::vector<bool>* isTrueCost)
{
  if (!m_using_lazy)
  {
    GetSuccs(0, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }
  GetLazySuccs(0, sourceStateID, succIDs, costs, isTrueCost);
}

void Environment::GetLazySuccs(int q_id, int sourceStateID, vector<int>* succIDs,
                           vector<int>* costs, std::vector<bool>* isTrueCost){

  if (!m_using_lazy)
  {
    GetSuccs(q_id, sourceStateID, succIDs, costs);
    isTrueCost->clear();
    isTrueCost->resize(succIDs->size(), 1);
    return;
  }

  double expansion_color = 250/NUM_SMHA_HEUR*q_id;
  vector<MotionPrimitivePtr> all_mprims = m_mprims.getMotionPrims();
    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================",
                    sourceStateID);

    succIDs->clear();
    succIDs->reserve(all_mprims.size());
    costs->clear();
    costs->reserve(all_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->robot_pose().visualize(expansion_color);
        //usleep(10000);
    }
    for (auto mprim : all_mprims){
        //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        //mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;

        if (mprim->motion_type() == MPrim_Types::ARM){
            successor.reset(new GraphState(*source_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());
            ROS_DEBUG_NAMED(SEARCH_LOG, "arm mprim/source/successor");
            mprim->printEndCoord();
            source_state->printToDebug(MPRIM_LOG);
            successor->printToDebug(MPRIM_LOG);
            ROS_DEBUG_NAMED(SEARCH_LOG, "done");
        } else {
            if (!mprim->apply(*source_state, successor, t_data)){
                //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
                continue;
            }
            ROS_DEBUG_NAMED(SEARCH_LOG, "non-arm mprim/source/successor");
            mprim->printEndCoord();
            source_state->printToDebug(MPRIM_LOG);
            successor->printToDebug(MPRIM_LOG);
            ROS_DEBUG_NAMED(SEARCH_LOG, "done");
        }
        m_hash_mgr->save(successor);
        Edge key;

        if (m_goal->isSatisfiedBy(successor)){
          m_goal->storeAsSolnState(successor);
          //ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
            //  mprim->cost());
          ROS_INFO("Found potential goal at: source->id %d, successor->id %d,"
            "cost: %d, mprim type: %d ", source_state->id(), successor->id(),
              mprim->cost(), mprim->motion_type());
          succIDs->push_back(GOAL_STATE);
          key = Edge(sourceStateID, GOAL_STATE);
        } else {
          succIDs->push_back(successor->id());
          key = Edge(sourceStateID, successor->id());
        }

//        succIDs->push_back(successor->id());
//        key = Edge(sourceStateID, successor->id());
        m_edges.insert(map<Edge, MotionPrimitivePtr>::value_type(key, mprim));
        costs->push_back(mprim->cost());
        isTrueCost->push_back(false);
    }
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int Environment::GetTrueCost(int parentID, int childID){
    TransitionData t_data;

    vector<MotionPrimitivePtr> small_mprims;
    if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
      ROS_ERROR("transition hasn't been found between %d and %d??", parentID, childID);
        assert(false);
    }
    small_mprims.push_back(m_edges[Edge(parentID, childID)]);
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

    ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
    GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
    GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
    GraphStatePtr successor;
    MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));
    if (!mprim->apply(*source_state, successor, t_data)){
        return -1;
    }
    mprim->printEndCoord();
    mprim->print();
    //source_state->printToInfo(SEARCH_LOG);
    //successor->printToInfo(SEARCH_LOG);
    successor->id(m_hash_mgr->getStateID(successor));

    // right before this point, the successor's graph state does not match the
    // stored robot state (because we modified the graph state without calling
    // ik and all that). this call updates the stored robot pose.
    real_next_successor->robot_pose(successor->robot_pose());

    bool matchesEndID = (successor->id() == childID) || (childID == GOAL_STATE);
    assert(matchesEndID);

    bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) &&
                            m_cspace_mgr->isValidTransitionStates(t_data));
    if (!valid_successor){
        return -1;
    }
    return t_data.cost();
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start,
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    m_start = start_pose;
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    m_edges.clear();

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    start_pose.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_pose);
    //m_cspace_mgr->visualizeCollisionModel(start_pose);
    //std::cin.get();

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();


    m_goal = search_request->createGoalState();

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    assert(goal_id == GOAL_STATE);
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);


    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.

    m_heur_mgr->initializeHeuristics();

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;

    readIslands();

    //Currently passing dummy island and activation center values. They get update later.
    std::vector<RobotState> islands, activationCenters;
    /* Moved to configureMotionPrimitive
    m_mprims = MotionPrimitivesMgr(m_goal, islands, activationCenters);

    // Choosed the snap mprims from launch file.
    chooseSnapMprims();
    */
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    // load up motion primitives
    //m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations.
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){

    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Setting planning mode to : %d",
        search_request->m_params->planning_mode);
    RobotState::setPlanningMode(search_request->m_params->planning_mode);

    m_state_time_map.clear();
    countSnapMprimsApplied.clear();
    countSnapMprimsSucceeded.clear();
    countSnapMprimsApplied.resize(3, 0);
    countSnapMprimsSucceeded.resize(3, 0);
}

void Environment::configureMotionPrimitives(SearchRequestPtr search_request) {
    //Delete mprims from last iteration.
    //m_mprims.clearMprims();

    //XXX Moved here from configurePlanningDomain
    std::vector<RobotState> islandStates, activationCenters;
    islandStates.assign(m_islandStates[0].begin(), m_islandStates[0].begin() + 110);
    activationCenters.assign(m_activationCenters[0].begin(), m_activationCenters[0].begin() + 110);
    ROS_ERROR("Size of islands list: %d", islandStates.size());

    ROS_WARN("Using simplified island selection.");
    //XXX The following function is the default one.
    //getIslandStates(islandStates, activationCenters);
    m_mprims = MotionPrimitivesMgr(m_goal, islandStates, activationCenters);
    chooseSnapMprims();
    //m_mprims.getUpdatedIslands(islandStates, activationCenters);

    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);
    ROS_ERROR("Mprims loaded.");

    m_mprims.getUpdatedGoalandTolerances(m_goal,
            search_request->m_params->xyz_tolerance,
            search_request->m_params->roll_tolerance,
            search_request->m_params->pitch_tolerance,
            search_request->m_params->yaw_tolerance);

    //We want two lists of mprims- one for anchor and another for inadmissible
    //heuristics and MHA* expands each state  twice, at max.
    m_mprims_inadmissible.clear();
    m_mprims_anchor.clear();
    for(auto mprim : m_mprims.getMotionPrims()) {
        m_mprims_inadmissible.push_back(mprim);
        m_mprims_anchor.push_back(mprim);
    }
    ROS_ERROR("Mprims bull");

    // This simulates giving each heuristic its own separate mprim list.
    //m_heuristic_fbs_mprimid.clear();
    //m_heuristic_base_mprimid.clear();
    //m_heuristic_fbs_mprimid.resize(20);
    //m_heuristic_base_mprimid.resize(20);

    //for(int i=0;i<20;i++) {
    //    for(auto mprim : m_mprims.getMotionPrims()) {
    //        if(mprim->motion_type() == MPrim_Types::FULLBODY_SNAP) {
    //            m_heuristic_fbs_mprimid[i].push_back(mprim->getID());
    //        }
    //        if(mprim->motion_type() == MPrim_Types::BASE_SNAP) {
    //            m_heuristic_base_mprimid[i].push_back(mprim->getID());
    //        }
    //    }
    //}

    countFbsMprimTried.clear();
    skipFbsMprimMod.clear();
    countBaseMprimTried.clear();
    skipBaseMprimMod.clear();

    //countFbsMprimTried.resize(m_numSnapMprims, 0);
    //skipFbsMprimMod.resize(m_numSnapMprims, 1);
    //countBaseMprimTried.resize(m_numSnapMprims, 0);
    //skipBaseMprimMod.resize(m_numSnapMprims, 1);

    ROS_ERROR("Motion prims configured.");
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path){
    //Save state time stats.
    save_state_time(soln_path);

    //save_heuristic_state_time(soln_path);

    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    std::vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, *m_goal, m_mprims.getMotionPrims());
    if(m_param_catalog.m_visualization_params.final_path){
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}

void Environment::generateStartState(SearchRequestPtr search_request) {
    ContObjectState start_obj_state(search_request->m_params->obj_start);
    ContBaseState base_start(search_request->m_params->base_start);
    RobotState start_robot_state(base_start, start_obj_state);
    start_robot_state.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_robot_state);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Generate start state : Keyboard");
    // std::cin.get();
    search_request->m_params->base_start = start_robot_state.getContBaseState();
    search_request->m_params->right_arm_start = start_robot_state.right_arm();
    search_request->m_params->left_arm_start = start_robot_state.left_arm();
}

//Saves the states in the path generated by a successful search by the planner,
//along with the time taken discover each state.. The time taken for the
//discovery of each state is found by calculating the difference between the
//time stamp of that state and the that of the previous.
void Environment::save_state_time(vector<int> soln_path) {
    ROS_INFO("Saving times to state.time.csv");

    ofstream file;
    file.open("./state_time.csv", std::fstream::app);

    for(int i=0;i<soln_path.size();i++) {
        int state_id = soln_path[i];
        GraphStatePtr state = m_hash_mgr->getGraphState(state_id);
        std::vector<double> right_arm;
        state->robot_pose().right_arm().getAngles(&right_arm);

        if(i==0) {
            file<<state_id;
            for(int j=0;j<right_arm.size();j++)
                file<<"\t"<<right_arm[j];
            file<<"\t"<<state->base_theta()<<"\t"<<state->base_x()<<"\t"<<state->base_y()<<"\t"<<state->base_z()<<"\t"<<0;
            file<<"\n";
        }
        else{
            file<<state_id;
            for(int j=0;j<right_arm.size();j++)
                file<<"\t"<<right_arm[j];
            file<<"\t"<<state->base_theta()<<"\t"<<state->base_x()<<"\t"<<state->base_y()<<"\t"<<state->base_z()<<"\t"<<m_state_time_map[state_id].second - m_state_time_map[soln_path[i-1]].second<<"\t"<<m_state_time_map[state_id].first<<"\n";
        }
    }
    file<<"$\n";
    file.close();

    ROS_INFO("File saved");
}

void Environment::saveStateHeuristics(std::vector<std::vector<std::pair<int, int>>> heurQueues) {
    std::ofstream outFile;
    outFile.open("./state_heuristics.csv", std::fstream::app);

    outFile<<"heuristic_queue\tcoords\theuristic_cost\n";
    for (int i=0; i<heurQueues.size(); i++) {
        auto queue = heurQueues[i];
        for (int j=0; j<queue.size(); j++) {
            int stateId = queue[j].first;
            int heurCost = queue[j].second;
            GraphStatePtr state = m_hash_mgr->getGraphState(stateId);
            std::vector<double> right_arm;
            state->robot_pose().right_arm().getAngles(&right_arm);

            outFile<<i;
            for(int j=0;j<right_arm.size();j++)
                outFile<<"\t"<<right_arm[j];
            outFile<<"\t"<<state->base_theta()<<"\t"<<state->base_x()<<"\t"<<state->base_y()<<"\t"<<state->base_z()<<"\t";

            outFile<<heurCost<<"\n";
        }
    }
    outFile<<"$\n";
    outFile.close();

    ROS_INFO("State heuristics File saved");
}

void Environment::normalize_heuristic_times() {
    for(int i=0;i<m_heuristic_state_time_map.size();i++) {
        double timeStart = m_heuristic_state_time_map[i][0].second;

        int mapLength = m_heuristic_state_time_map[i].size();
        double deltaTime = 0;
        double deltaTime_ = 0;
        std::pair<int, double> j_elem, j_elem_;
        for(int j=1;j<mapLength;j++) {
            j_elem = m_heuristic_state_time_map[i][j];
            j_elem_ = m_heuristic_state_time_map[i][j-1];

            deltaTime_ = j_elem.second - j_elem_.second;
            deltaTime += deltaTime_;
            m_heuristic_state_time_map[i][j] = std::make_pair(j_elem.first, deltaTime);
        }
    }
}

void Environment::save_heuristic_state_time(vector<int> soln_path) {
    ROS_INFO("Saving times to state.time.csv");

    ofstream file;
    file.open("heuristic_state_time.csv", std::fstream::app);

    normalize_heuristic_times();

    for(int j=0; j<m_heuristic_state_time_map.size();j++) {
        ROS_ERROR("Heuristic state size = %d", m_heuristic_state_time_map[j].size());
        int k = 0, l=0;
        std::pair<int, double> k_elem;
        double time_of_previous_heuristic_state_in_soln=0;
        while(k < m_heuristic_state_time_map[j].size() && l < soln_path.size()) {
            int state_id = soln_path[l];
            k_elem = m_heuristic_state_time_map[j][k];

            if(k_elem.first < state_id) {
                k++;
            }
            else if(k_elem.first > state_id)
                l++;
            else {
                // This state id is in the soln path.
                GraphStatePtr state = m_hash_mgr->getGraphState(state_id);

                if(k==0) {
                    file<<std::setw(10)<<state_id<<"\t"<<j<<"\t"<<state->obj_x()<<"\t"<<state->obj_y()<<"\t"<<state->obj_z()<<"\t"<<state->obj_roll()<<"\t"<<state->obj_pitch()<<"\t"<<state->obj_yaw()<<"\t"<< state->base_theta()<<"\t"<<state->base_x()<<"\t"<<state->base_y()<<"\t"<<0<<"\n";
                    time_of_previous_heuristic_state_in_soln = 0;
                }
                else {
                    file<<std::setw(10)<<state_id<<"\t"<<j<<"\t"<<state->obj_x()<<"\t"<<state->obj_y()<<"\t"<<state->obj_z()<<"\t"<<state->obj_roll()<<"\t"<<state->obj_pitch()<<"\t"<<state->obj_yaw()<<"\t"<< state->base_theta()<<"\t"<<state->base_x()<<"\t"<<state->base_y()<<"\t"<<k_elem.second - time_of_previous_heuristic_state_in_soln<<"\n";
                    time_of_previous_heuristic_state_in_soln = k_elem.second;
                }
                k++, l++;
            }
        }
    }
    file.close();

    ROS_INFO("File saved");
}

void Environment::readIslands() {
    std::string file_name;
    ROS_INFO("Loading bottleneck points");
    m_nodehandle.param("planner/island", file_name, std::string("Empty"));
    ROS_INFO("Base island file name %s", file_name.c_str());

    bool visualizeIslands;
    m_nodehandle.param("planner/visualizeIslands", m_visualizeIslands, false);
    ROS_INFO("Visualize islands option set to: %d", m_visualizeIslands);

    ifstream island_file(file_name);

    std::string line;
    std::string word;
    double q_w, q_x, q_y, q_z;
    double pitch, roll;
    double x, y, yaw, z;
    std::vector<double> rarm;
    rarm.resize(7);
    std::vector<double> larm = {0.038946, 1.214670, 1.396356, -1.197227, -4.616317, -0.988727, 1.175568};
    LeftContArmState l_arm(larm);

    KDL::Rotation startOrien, goalOrien;

    std::getline(island_file, line);
    while(std::getline(island_file, line)) {
        //std::istringstream ss(line);
        //ss >> x >> y >> z >> q_w >> q_x >> q_y >> q_z;
        //startOrien = KDL::Rotation::Quaternion(q_w, q_x, q_y, q_z);
        //startOrien.GetRPY(roll, pitch, yaw);
        //ContObjectState start(x, y, z, roll, pitch, yaw);

        //std::getline(island_file, line);
        //std::istringstream ss1(line);
        //ss1 >> x >> y >> z >> q_w >> q_x >> q_y >> q_z;
        //goalOrien = KDL::Rotation::Quaternion(q_w, q_x, q_y, q_z);
        //goalOrien.GetRPY(roll, pitch, yaw);
        //ContObjectState goal(x, y, z, roll, pitch, yaw);

        //m_startGoalPairs.push_back(std::make_pair(start, goal));

        //std::getline(island_file, line);
        //ROS_INFO("%s", line.c_str());
        //std::istringstream ss2(line);
        //ss2 >> word;

        std::vector<RobotState> islandStates, activationCenters;

        //Start-goal points.
        while(word[0] != 'I') {
            std::istringstream ssData(line);

            //for(int i=0;i<2;i++) {

                for(int j=0;j<7;j++) {
                    ssData >> rarm[j];
                }
                ssData >>yaw >> x >> y >> z;

                double theta_res = m_param_catalog.m_robot_resolution_params.base_theta_resolution;
                yaw = normalize_angle_positive(static_cast<double>(yaw)*theta_res);

                const ContBaseState base(x, y, z, yaw);
                RightContArmState r_arm(rarm);

                RobotState islandState(base, r_arm, l_arm);
                islandStates.push_back(islandState);
                activationCenters.push_back(islandState);
                /*
                if(i==0) {
                    RobotState islandState(base, r_arm, l_arm);
                    islandStates.push_back(islandState);
                }
                else {
                    RobotState activationCenter(base, r_arm, l_arm);
                    activationCenters.push_back(activationCenter);
                }
                */
            //}
            if(!std::getline(island_file, line))
                break;
            //ROS_INFO("%s", line.c_str());
            std::istringstream ssa(line);
            ssa >> word;
        }
        m_islandStates.push_back(islandStates);
        m_activationCenters.push_back(activationCenters);
    }
    //ROS_INFO("DONE");
}

// Using euclidean distance, give a distance between two objectstates.
double objectStateMetric( ContObjectState a,  ContObjectState b) {
    double distance = 0;

    std::vector<double> values_a, values_b;
    a.getStateValues(&values_a);
    b.getStateValues(&values_b);

    // For x,y,z coordinates.
    for(int i=0; i< 3; i++) {
        distance += (values_a[i] - values_b[i])*(values_a[i] - values_b[i]);
    }

     // We don't want the orientation of the object to influence the metric
     // much. The x, y, z coordinates are in meters the metric much
    float factor = 0;
    // For roll, pitch, yaw.
    for(int i=3; i<6; i++) {
        distance += factor * abs(angles::shortest_angular_distance(values_a[i], values_b[i]));
    }

    return distance;
}

int findMaxIndex(std::vector<std::pair< int, double> > distances) {
    int maxI = 0;
    for(int i=0;i < distances.size();i++) {
        if(distances[i].second > distances[maxI].second)
            maxI = i;
    }
    return maxI;
}

void visualizeCircle(std::pair<double, double> center, double radius, std::string ns) {
    int numPoints = int(100*radius);
    float resolution = 2*3.14 / numPoints;

    std::vector<std::vector<double> > circlePoints;
    double x=0, y=0;
    double theta = 0;

    for(int i=0;i<numPoints;i++) {
        theta += resolution;
        x = center.first +  radius * cos(theta);
        y = center.second +  radius * sin(theta);
        circlePoints.push_back(std::vector<double>({x, y, 0}));
    }
    PViz pviz;
    pviz.visualize3DPath(circlePoints, ns);

}

//Return the islands and activation centers corresponding to the closest start-goal pairs that we trained on. This way we dont waste time in using unrelated island states.
void Environment::getIslandStates(std::vector<RobotState> &islandStates, std::vector<RobotState> &activationCenters) {
    ContObjectState startObj, goalObj;
    startObj = m_start.getObjectStateRelMap();
    goalObj = m_goal->getObjectState();

    std::vector<double> startGoalDistances;

    double distance;
    //ROS_ERROR("Printing start goal pairs");
    for(int i=0;i < m_startGoalPairs.size();i++) {
        // Give more weightage to goal.
        startGoalDistances.push_back(objectStateMetric(startObj, m_startGoalPairs[i].first) + objectStateMetric(goalObj, m_startGoalPairs[i].second));
        ROS_INFO("%f, %f", objectStateMetric(startObj, m_startGoalPairs[i].first), objectStateMetric(goalObj, m_startGoalPairs[i].second));
        //startGoalDistances.push_back(objectStateMetric(goalObj, m_startGoalPairs[i].second));
        //ROS_INFO("(%f, %f), (%f, %f), %f", m_startGoalPairs[i].second.x(), m_startGoalPairs[i].second.y(), goalObj.x(), goalObj.y(), startGoalDistances[i]);
    }

    int numClosestPairs = 1; //4
    int numIslandsPerPair = 10;  //20
    m_numSnapMprims = numClosestPairs * numIslandsPerPair;
    //Index-distance.
    std::vector<std::pair<int, double> > closestPairIndices;

    for(int i=0;i<numClosestPairs;i++) {
        closestPairIndices.push_back(make_pair(i, startGoalDistances[i]));
    }

    int maxIndex=0;
    for(int i=numClosestPairs;i<m_startGoalPairs.size(); i++) {
        maxIndex = findMaxIndex(closestPairIndices);
        if(startGoalDistances[i] < closestPairIndices[maxIndex].second) {
            closestPairIndices[maxIndex] = make_pair(i, startGoalDistances[i]);
        }
    }

    std::vector<RobotState> pairIslandStates;
    std::vector<RobotState> pairActivationCenters;
    //ROS_ERROR("Closest islands");
    for(int i=0;i < closestPairIndices.size();i++) {
        //ROS_INFO("%d, %f", closestPairIndices[i].first, closestPairIndices[i].second);

        pairIslandStates = m_islandStates[closestPairIndices[i].first];
        islandStates.insert(islandStates.end(), pairIslandStates.begin(), pairIslandStates.begin() + numIslandsPerPair);

        pairActivationCenters = m_activationCenters[closestPairIndices[i].first];
        activationCenters.insert(activationCenters.end(), pairActivationCenters.begin(), pairActivationCenters.begin() + numIslandsPerPair);
    }


    if(m_visualizeIslands) {
        PViz pviz;
        for(int j=0;j<islandStates.size();j++) {
                RobotState state = islandStates[j];
                std::vector<double> base({state.getContBaseState().x(), state.getContBaseState().y(), state.getContBaseState().theta()});
                std::vector<double> rarm, larm;
                state.right_arm().getAngles(&rarm);
                state.left_arm().getAngles(&larm);
                string ns = "wee" + string("%d", j);

                pviz.visualizeRobot(rarm, larm, base, 0.1, 140, ns, 0, false);
                //visualizeCircle(std::make_pair(base[0], base[1]), 0.7, ns);
        }
    }
}


void Environment::chooseSnapMprims() {
    int baseSnap, fullBodySnap, armSnap;
    ROS_INFO("Setting snap mprim types from launch file");
    m_nodehandle.param("planner/baseSnap", baseSnap, 0);
    m_nodehandle.param("planner/fullBodySnap", fullBodySnap, 0);
    m_nodehandle.param("planner/armSnap", armSnap, 0);
    m_mprims.baseSnap = int(baseSnap);
    m_mprims.fullBodySnap = int(fullBodySnap);
    m_mprims.armSnap = int(armSnap);
    ROS_INFO("%d\t%d\t%d\n", int(baseSnap), int(fullBodySnap), int(armSnap));
}








