// Component
#include "SplineHelper.hpp"

namespace local_planner
{

float64_t SplineHelper::calcTargetHeadingR(const GraphNode& current_node, const GraphNode& goal_node, const float64_t spline_order, const float64_t time_step_ms)
{
    const Spline1d target_spline = SplineHelper().calcTargetSpline(current_node, goal_node, spline_order);

    const float64_t dx_m = current_node.getEstimatedPointM().getX() + current_node.getEstimatedLongitudinalVelocityMps() * time_step_ms / 1000.0;
    const float64_t dy_m = current_node.getEstimatedPointM().getY() + current_node.getEstimatedLateralVelocityMps()      * time_step_ms / 1000.0;
    const float64_t dist_m = std::sqrt(std::pow(dx_m, 2U) + std::pow(dy_m, 2U));

    constexpr float64_t spline_res = 0.05;
    float64_t spline_value{0.0};
    float64_t spline_dist_m{0.0};
    float64_t x_m;
    float64_t y_m;
    float64_t prev_x_m = current_node.getEstimatedPointM().getX();
    float64_t prev_y_m = current_node.getEstimatedPointM().getY();

    while(spline_dist_m < dist_m)
    {
        const Eigen::MatrixXd spline_pt = target_spline(spline_value);
        x_m = spline_pt(0U);
        y_m = spline_pt(1U);

        const float64_t dp_m = std::sqrt(std::pow(x_m - prev_x_m, 2U) + std::pow(y_m - prev_y_m, 2U));

        spline_dist_m += dp_m;
        spline_value  += spline_res;
    }
    
    return std::atan2(y_m - current_node.getEstimatedPointM().getY(), x_m - current_node.getEstimatedPointM().getY());
}

Spline1d SplineHelper::calcTargetSpline(const GraphNode& current_node, const GraphNode& goal_node, const float64_t spline_order)
{
    const float64_t start_x_m  = current_node.getEstimatedPointM().getX();
    const float64_t start_y_m  = current_node.getEstimatedPointM().getY();

    const float64_t end_heading_r = goal_node.getEstimatedHeadingR();
    const float64_t end_x_m       = goal_node.getEstimatedPointM().getX();
    const float64_t end_y_m       = goal_node.getEstimatedPointM().getY();
    constexpr float64_t dp_m      = 0.001;
    const float64_t dx_m          = std::copysign(dp_m, end_x_m);
    const float64_t end_x_m_      = end_x_m - dx_m * std::cos(end_heading_r);
    const float64_t dy_m          = std::copysign(dp_m, end_y_m);
    const float64_t end_y_m_      = end_y_m - dy_m * std::sin(end_heading_r);

    Eigen::MatrixXd points(2U, 3U);
    points << start_x_m, end_x_m_, end_x_m,
              start_y_m, end_y_m_, end_y_m;

    const Spline1d spline = Spline1dFitting::Interpolate(points, spline_order);

    return spline;
}

} // namespace local_planner
