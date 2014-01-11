#pragma once
#include <arm_navigation_msgs/CollisionMap.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <monolithic_pr2_planner/Heuristics/HeuristicMgr.h>
#include <nav_msgs/OccupancyGrid.h>

namespace monolithic_pr2_planner_node {
    typedef tf::MessageFilter<arm_navigation_msgs::CollisionMap> CollisionMapMsgFilter;
    typedef std::vector<Eigen::Vector3d> VoxelList;
    class CollisionSpaceInterface {
        public:
            CollisionSpaceInterface(monolithic_pr2_planner::CSpaceMgrPtr, 
                                    monolithic_pr2_planner::HeuristicMgrPtr);
            bool bindCollisionSpaceToTopic(std::string topic_name, 
                                           tf::TransformListener& tf, 
                                           std::string ref_frame);
            void update3DHeuristicMaps();
            void update2DHeuristicMaps(std::vector<signed char>& data);

        private:
            std::string m_ref_frame;
            void mapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);
            message_filters::Subscriber<arm_navigation_msgs::CollisionMap> m_collision_map_subscriber;
            boost::shared_ptr<CollisionMapMsgFilter> m_collision_map_filter;
            monolithic_pr2_planner::CSpaceMgrPtr m_cspace_mgr;
            ros::NodeHandle m_nodehandle;
            ros::Publisher m_cmap_pub;
            monolithic_pr2_planner::HeuristicMgrPtr m_heur_mgr;
    };
}
