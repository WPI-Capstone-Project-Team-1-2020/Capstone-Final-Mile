// Component
#include "TrajectorySolver.hpp"

// Ros

// Libraries
#include <boost/cstdfloat.hpp>
#include <boost/shared_ptr.hpp>

// Standard
#include <utility>

namespace local_planner
{

TrajectorySolver::TrajectorySolver(const std::shared_ptr<LocalPlannerConfig>& cfg) :
    m_cfg{cfg}
{}

TrajectorySolver::~TrajectorySolver() = default;

autonomy_msgs::Trajectory::ConstPtr TrajectorySolver::calculateTrajectory(const std::vector<Point>& path)
{

}

} // namespace local_planner