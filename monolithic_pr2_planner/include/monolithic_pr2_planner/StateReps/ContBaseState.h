#pragma once
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <vector>
namespace monolithic_pr2_planner {
    class DiscBaseState;
    class ContBaseState : public OccupancyGridUser {
        public:
            ContBaseState();
            ContBaseState(double x, double y, double z, double theta);
            ContBaseState(std::vector<double> base_pose);
            ContBaseState(const DiscBaseState& base_pose);

            void getValues(std::vector<double>* values) { *values = m_pose; };
            double x() const { return m_pose[BodyDOF::X]; };
            double y() const { return m_pose[BodyDOF::Y]; };
            double z() const { return m_pose[BodyDOF::Z]; };
            double theta() const { return m_pose[BodyDOF::THETA]; };

            void x(double x) { m_pose[BodyDOF::X] = x; };
            void y(double y) { m_pose[BodyDOF::Y] = y; };
            void z(double z) { m_pose[BodyDOF::Z] = z; };
            void theta(double theta) { m_pose[BodyDOF::THETA] = normalize_angle_positive(theta); };

            DiscBaseState getDiscBaseState();

            static bool interpolate(const ContBaseState& start, const ContBaseState& end,
                                    int num_steps, std::vector<ContBaseState>* interp_steps);
        private:
            std::vector<double> m_pose;
    };
}
