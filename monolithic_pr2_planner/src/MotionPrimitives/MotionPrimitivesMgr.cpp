#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/foreach.hpp>
using namespace monolithic_pr2_planner;
using namespace std;
using namespace boost;

MotionPrimitivesMgr::MotionPrimitivesMgr(boost::shared_ptr<GoalState>& goal) : m_all_mprims(8){m_goal = goal; }

// Pass island states to MotionPrimitiveMgs from Environment.
MotionPrimitivesMgr::MotionPrimitivesMgr(const boost::shared_ptr<GoalState>& goal, const std::vector<RobotState> &islandStates, const std::vector<RobotState> &activationCenters) : m_all_mprims(8), m_goal(goal) {m_islandStates = islandStates, m_activationCenters = activationCenters;}

/*! \brief loads all mprims from configuration. also sets up amps. note that
 * these are not necessarily the exact mprims used during search, because
 * there's a user parameter that selects which mprims to actually use. 
 */
bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveParams& params){
    m_params = params;
    
    MPrimList arm_mprims;
    m_parser.parseArmMotionPrimitives(params.arm_motion_primitive_file, arm_mprims);

    MPrimList base_mprims;
    m_parser.parseBaseMotionPrimitives(params.base_motion_primitive_file, base_mprims);
    
    
    ArmAdaptiveMotionPrimitivePtr armAMP = make_shared<ArmAdaptiveMotionPrimitive>();
   
    ArmTuckMotionPrimitivePtr tuckAMP = make_shared<ArmTuckMotionPrimitive>();
    ArmUntuckMotionPrimitivePtr untuckAMP = make_shared<ArmUntuckMotionPrimitive>(true);
    ArmUntuckMotionPrimitivePtr untuckPartialAMP = make_shared<ArmUntuckMotionPrimitive>(false);

    MPrimList arm_amps;
    arm_amps.push_back(armAMP);
    arm_amps.push_back(tuckAMP);
    arm_amps.push_back(untuckAMP);
    //arm_amps.push_back(untuckPartialAMP);

    MPrimList base_amps;
    int NEG_TURN = -1;
    int POS_TURN = 1;
    BaseAdaptiveMotionPrimitivePtr bamp1 = make_shared<BaseAdaptiveMotionPrimitive>(NEG_TURN);
    BaseAdaptiveMotionPrimitivePtr bamp2 = make_shared<BaseAdaptiveMotionPrimitive>(POS_TURN);
    base_amps.push_back(bamp1);
    base_amps.push_back(bamp2);

    MPrimList torso_mprims;
    int VERTICAL_UP = 1;
    int VERTICAL_DOWN = -1;
    TorsoMotionPrimitivePtr t_mprim1 = make_shared<TorsoMotionPrimitive>(VERTICAL_UP);
    TorsoMotionPrimitivePtr t_mprim2 = make_shared<TorsoMotionPrimitive>(VERTICAL_DOWN);
    torso_mprims.push_back(t_mprim1);
    torso_mprims.push_back(t_mprim2);

    // Base snap mprims.
    MPrimList base_snap_mprims;
    if(baseSnap) {
        for(int i=0;i<m_islandStates.size();i++) {
            BaseSnapMotionPrimitivePtr temp = make_shared<BaseSnapMotionPrimitive>(m_islandStates[i]);
            temp->setID(i);
            ContBaseState islandBase, activationBase;
            islandBase = m_islandStates[i].getContBaseState();
            activationBase = m_activationCenters[i].getContBaseState();
            ContBaseState baseActivationRadius(abs(islandBase.x() - activationBase.x()), abs(islandBase.y() - activationBase.y()), abs(islandBase.z() - activationBase.z()), abs(islandBase.theta() - activationBase.theta()));

            std::vector<double> armActivationRadius, islandArm, activationCenterArm;
            m_islandStates[i].right_arm().getAngles(&islandArm);
            m_activationCenters[i].right_arm().getAngles(&activationCenterArm);

            for(int j=0;j<7;j++) {
                armActivationRadius.push_back(abs(angles::shortest_angular_distance(islandArm[j], activationCenterArm[j])));
            }

            std::vector<double> l_arm = {0.038946, 1.214670, 1.396356, -1.197227, -4.616317, -0.988727, 1.175568};

            RobotState activationRadius(baseActivationRadius, RightContArmState(armActivationRadius), LeftContArmState(l_arm));
            temp->m_activationRadius = activationRadius;
            basesnap_mprim.push_back(temp);
            base_snap_mprims.push_back(temp);
        }
    }

    MPrimList full_body_snap_mprims;
    if(fullBodySnap) {
        for(int i=0;i<m_islandStates.size();i++) {
            FullBodySnapMotionPrimitivePtr temp = make_shared<FullBodySnapMotionPrimitive>(m_islandStates[i]);
            temp->setID(i);

            ContBaseState islandBase, activationBase;
            islandBase = m_islandStates[i].getContBaseState();
            activationBase = m_activationCenters[i].getContBaseState();
            //ROS_ERROR("Base(island, activation): (%f, %f)", islandBase.x(), activationBase.x());
            ContBaseState baseActivationRadius(abs(islandBase.x() - activationBase.x()), abs(islandBase.y() - activationBase.y()), abs(islandBase.z() - activationBase.z()), abs(angles::shortest_angular_distance(islandBase.theta(), activationBase.theta())));

            std::vector<double> armActivationRadius, islandArm, activationCenterArm;
            m_islandStates[i].right_arm().getAngles(&islandArm);
            m_activationCenters[i].right_arm().getAngles(&activationCenterArm);

            for(int j=0;j<7;j++) {
                armActivationRadius.push_back(abs(angles::shortest_angular_distance(islandArm[j], activationCenterArm[j])));
            }

            std::vector<double> l_arm = {0.038946, 1.214670, 1.396356, -1.197227, -4.616317, -0.988727, 1.175568};

            RobotState activationRadius(baseActivationRadius, RightContArmState(armActivationRadius), LeftContArmState(l_arm));
            DiscBaseState db = activationRadius.base_state();
            temp->m_activationRadius = activationRadius;
            fullbody_snap_mprim.push_back(temp);
            full_body_snap_mprims.push_back(temp);
        }
    }

    // Arm snap mprims.
    MPrimList arm_snap_mprims;
    if(armSnap) {
        armsnap_mprim = boost::make_shared<ArmSnapMotionPrimitive>();
        arm_snap_mprims.push_back(armsnap_mprim);
    }

    m_all_mprims[MPrim_Types::ARM] = arm_mprims;
    m_all_mprims[MPrim_Types::BASE] = base_mprims;
    m_all_mprims[MPrim_Types::TORSO] = torso_mprims;
    m_all_mprims[MPrim_Types::ARM_ADAPTIVE] = arm_amps;
    m_all_mprims[MPrim_Types::BASE_ADAPTIVE] = base_amps;
    
    if(baseSnap)
        m_all_mprims[MPrim_Types::BASE_SNAP] = base_snap_mprims;
    if(fullBodySnap)
        m_all_mprims[MPrim_Types::FULLBODY_SNAP] = full_body_snap_mprims;
    if(armSnap)
        m_all_mprims[MPrim_Types::ARM_SNAP] = arm_snap_mprims;

    computeAllMPrimCosts(m_all_mprims);

    loadAllMPrims();

    for (auto& mprim: m_active_mprims){    
        mprim->print();
    }

    return true;
}

void MotionPrimitivesMgr::loadMPrimSet(int planning_mode){
    m_active_mprims.clear();
    bool is_arm_only = (planning_mode == PlanningModes::RIGHT_ARM || 
                        planning_mode == PlanningModes::LEFT_ARM || 
                        planning_mode == PlanningModes::DUAL_ARM);
    bool is_mobile = (planning_mode == PlanningModes::RIGHT_ARM_MOBILE || 
                      planning_mode == PlanningModes::LEFT_ARM_MOBILE || 
                      planning_mode == PlanningModes::DUAL_ARM_MOBILE);
    if (planning_mode == PlanningModes::BASE_ONLY){
        loadBaseOnlyMPrims();
    } else if (is_arm_only){
        loadArmOnlyMPrims();
    } else if (is_mobile){
        loadAllMPrims();
    } else {
        ROS_ERROR("Invalid planning mode!");
        assert(false);
    }
}

void MotionPrimitivesMgr::getUpdatedGoalandTolerances(const GoalStatePtr& goal,
        const double xyz_tol, const double roll_tol, const double pitch_tol,
        const double yaw_tol) {
    m_goal = goal;

    RobotPosePtr islandState;
    for(int i=0;i < m_islandStates.size();i++) {
        islandState = boost::make_shared<RobotState>(m_islandStates[i]);
        if(baseSnap) {
            basesnap_mprim[i]->getUpdatedGoalandTolerances(islandState, xyz_tol, roll_tol, pitch_tol, yaw_tol);
            basesnap_mprim[i]->m_end = goal;
            basesnap_mprim[i]->m_activationCenter = m_activationCenters[i];
        }
        if(fullBodySnap) {
            fullbody_snap_mprim[i]->getUpdatedGoalandTolerances(islandState, xyz_tol, roll_tol, pitch_tol, yaw_tol);
            fullbody_snap_mprim[i]->m_end = goal;
            fullbody_snap_mprim[i]->m_activationCenter = m_activationCenters[i];
        }
    }

    if(armSnap)
        armsnap_mprim->getUpdatedGoalandTolerances(m_goal, xyz_tol, roll_tol, pitch_tol, yaw_tol);

}
void MotionPrimitivesMgr::combineVectors(const MPrimList& v1, MPrimList& v2){
    for (auto& mprim : v1){
        v2.push_back(mprim);
    }
}

void MotionPrimitivesMgr::loadBaseOnlyMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::BASE], m_active_mprims);
    combineVectors(m_all_mprims[MPrim_Types::BASE_ADAPTIVE], m_active_mprims);
}

void MotionPrimitivesMgr::loadTorsoMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::TORSO], m_active_mprims);
}

// note that we don't separate left and right arm mprims here, since the mprims
// are in cartesian space
void MotionPrimitivesMgr::loadArmOnlyMPrims(){
    combineVectors(m_all_mprims[MPrim_Types::ARM], m_active_mprims);
    combineVectors(m_all_mprims[MPrim_Types::ARM_ADAPTIVE], m_active_mprims);
}

void MotionPrimitivesMgr::loadBaseSnapMPrims(){
     combineVectors(m_all_mprims[MPrim_Types::BASE_SNAP], m_active_mprims);
}

void MotionPrimitivesMgr::loadFullBodySnapMPrims(){
     combineVectors(m_all_mprims[MPrim_Types::FULLBODY_SNAP], m_active_mprims);
}

void MotionPrimitivesMgr::loadArmSnapMPrims() {
    combineVectors(m_all_mprims[MPrim_Types::ARM_SNAP], m_active_mprims);
}

void MotionPrimitivesMgr::loadAllMPrims(){
    loadBaseOnlyMPrims();
    loadArmOnlyMPrims();
    loadTorsoMPrims();
    
    if(baseSnap)
        loadBaseSnapMPrims();
    if(fullBodySnap)
        loadFullBodySnapMPrims();
    if(armSnap)
        loadArmSnapMPrims();
}

void MotionPrimitivesMgr::computeAllMPrimCosts(vector<MPrimList> mprims){
    for (auto& mprim_list : mprims){
        for (auto& mprim : mprim_list){
            mprim->computeCost(m_params);
        }
    }
}

void MotionPrimitivesMgr::updateParams(MotionPrimitiveParams params) {
    for(auto mprim: basesnap_mprim) { mprim->updateParams(params);
    }
}

void MotionPrimitivesMgr::addIslandSnapPrimitives() {
    loadBaseSnapMPrims();
}

void MotionPrimitivesMgr::deleteMPrim(int id, int type) {
    //Assumes that id is the index of the mprim in m_all_mprims.
    for(int i=0;i<m_active_mprims.size();i++) {
        auto mprim = m_active_mprims[i];
        if(mprim->motion_type() == type && mprim->getID() == id) {
            m_active_mprims.erase(std::remove(m_active_mprims.begin(), m_active_mprims.end(), mprim), m_active_mprims.end());
            break;
        }
    }
}
