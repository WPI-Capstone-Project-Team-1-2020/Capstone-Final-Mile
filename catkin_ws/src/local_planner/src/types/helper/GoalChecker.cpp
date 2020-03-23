// Component
#include "GoalChecker.hpp"

namespace local_planner
{

bool GoalChecker::checkGoalReached(const LocalPlannerData& data, const float64_t tol_m) noexcept
{
    const float64_t dx_m = data.getGoalPose()->x_m - data.getLocalPose()->pose.pose.position.x;
    const float64_t dy_m = data.getGoalPose()->y_m - data.getLocalPose()->pose.pose.position.y;

    const float64_t dp_m = std::sqrt(std::pow(dx_m, 2U) + std::pow(dy_m, 2U));

    return (dp_m <= tol_m);
}

} // namespace local_planner