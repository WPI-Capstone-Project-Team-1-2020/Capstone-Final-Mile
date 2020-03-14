// Component
#include "AStar.hpp"
#include "ForwardSimHelper.hpp"
#include "LocalPlannerConfig.hpp"
#include "RosConversionHelper.hpp"
#include "SplineHelper.hpp"

// Ros
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// Standard
#include <utility>

namespace local_planner
{

std::uint64_t GraphNode::id_generator{0U};
GraphNodeToleranceConfig GraphNode::tolerance_cfg{};

AStar::AStar(const std::shared_ptr<LocalPlannerConfig>& cfg) :
    m_cfg{cfg},
    m_open_nodes{cfg->getNodeDensityConfig()}
{    
    GraphNode::tolerance_cfg = m_cfg->getGraphNodeToleranceConfig();    
}

AStar::~AStar(){}

bool AStar::update()
{
    resetPlanner();
    initializePlanner();
    return planTrajectory();
}

void AStar::resetPlanner() noexcept
{
   m_frontier   = std::priority_queue<GraphNode, std::vector<GraphNode>>();
   m_open_nodes = NodeDensityGrid(m_cfg->getNodeDensityConfig());    
   m_goal_node  = GraphNode();
   GraphNode::id_generator = 0U;
}

void AStar::initializePlanner() noexcept
{
    GraphNode start_node;    
    start_node.setEstimatedPointM(Point(m_data.getLocalPose()->pose.pose.position.x, m_data.getLocalPose()->pose.pose.position.y));
    start_node.setEstimatedLongitudinalVelocityMps(3.0);//m_data.getLocalPose()->twist.twist.linear.x);
    start_node.setEstimatedLateralVelocityMps(m_data.getLocalPose()->twist.twist.linear.y);
    start_node.setEstimatedHeadingR(RosConversionHelper::quaternionMsgToYawR(m_data.getLocalPose()->pose.pose.orientation));
    start_node.setEstimatedYawRateRps(m_data.getLocalPose()->twist.twist.angular.z);    
    start_node.setCost(std::numeric_limits<float64_t>::max());

    m_frontier.emplace(start_node);
    m_open_nodes.setGridOrigin(start_node.getEstimatedPointM());
    static_cast<void>(m_open_nodes.addNodeToOpenSet(start_node));

    m_goal_node.setEstimatedPointM(Point(m_data.getGoalPose()->x_m, m_data.getGoalPose()->y_m));
    m_goal_node.setEstimatedLongitudinalVelocityMps(m_data.getGoalPose()->longitudinal_velocity_mps);
    m_goal_node.setEstimatedLateralVelocityMps(m_data.getGoalPose()->lateral_velocity_mps);
    m_goal_node.setEstimatedHeadingR(m_data.getGoalPose()->heading_r);
    m_goal_node.setEstimatedYawRateRps(m_data.getGoalPose()->yaw_rate_rps);
}

bool AStar::planTrajectory()
{  
    while (true)
    {
        if (m_frontier.empty() == false)
        {
            GraphNode current_node = m_frontier.top();

            // std::cout << "X: " << current_node.getEstimatedPointM().getX() 
            // << " Y: " << current_node.getEstimatedPointM().getY() 
            // << " Lateral: " << current_node.getEstimatedLateralVelocityMps()
            // << " Lon: "<< current_node.getEstimatedLongitudinalVelocityMps()
            // << " Head: " << current_node.getEstimatedHeadingR()
            // << " Yaw Rate: " << current_node.getEstimatedYawRateRps() << std::endl;

            if (current_node == m_goal_node)
            {
                ROS_INFO_STREAM("Path found!");
                return true;
            }

            m_frontier.pop();
            expandFrontier(current_node);
        }
        else
        {
            ROS_ERROR_STREAM("Frontier empty, all possible nodes explored");
            return false;
        }
    }
}

void AStar::expandFrontier(const GraphNode& current_node)
{
    std::vector<GraphNode> neighbors = calcNeighbors(current_node);
    
    std::for_each(neighbors.cbegin(),
        neighbors.cend(),
        [&frontier = this->m_frontier, &open_nodes = this->m_open_nodes](const GraphNode& node) -> void
        {
            if (open_nodes.addNodeToOpenSet(node) == true)
            {             
                frontier.emplace(node);
            }
        });
    
    m_open_nodes.closeNode(current_node);
}

std::vector<GraphNode> AStar::calcNeighbors(const GraphNode& current_node)
{
    const float64_t target_heading_r = 0;//SplineHelper::calcTargetHeadingR(current_node, m_goal_node, m_cfg->getSplineOrder(), m_cfg->getTimeStepMs());

    std::vector<float64_t> possible_lon_velocities_mps = calcPossibleLongitudinalVelocitiesMps(current_node);
    std::vector<float64_t> possible_lat_velocities_mps = calcPossibleLateralVelocitiesMps(current_node);
    std::vector<float64_t> possible_yaw_rates_rps      = calcPossibleYawRatesRps(current_node);

    std::vector<GraphNode> neighbors;
    neighbors.reserve(possible_lon_velocities_mps.size()*possible_lat_velocities_mps.size()*possible_yaw_rates_rps.size());

    std::for_each(possible_lon_velocities_mps.cbegin(),
        possible_lon_velocities_mps.cend(),
        [&neighbors, &possible_lat_velocities_mps, &possible_yaw_rates_rps, &current_node, target_heading_r, this](const float64_t x_vel_mps) -> void
        {
            std::for_each(possible_lat_velocities_mps.cbegin(),
                possible_lat_velocities_mps.cend(),
                [&neighbors, x_vel_mps, &possible_yaw_rates_rps, &current_node, target_heading_r, this](const float64_t y_vel_mps) -> void
                {                    
                    if (std::sqrt(std::pow(x_vel_mps, 2U) + std::pow(y_vel_mps, 2U)) <= m_cfg->getMaxVelMps())
                    {
                        std::for_each(possible_yaw_rates_rps.cbegin(),
                            possible_yaw_rates_rps.cend(),
                            [&neighbors, x_vel_mps, y_vel_mps, &current_node, target_heading_r, this](const float64_t yaw_rate_rps) -> void
                            {
                                GraphNode node = current_node;
                                node.setParentID(current_node.getID());
                                node.setEstimatedLongitudinalVelocityMps(x_vel_mps);
                                node.setEstimatedLateralVelocityMps(y_vel_mps);
                                node.setEstimatedYawRateRps(yaw_rate_rps);                          

                                ForwardSimHelper::forwardSimGraphNode(node, current_node, m_cfg->getTimeStepMs());

                                node.setG(calcNodeGScore(current_node, node));
                                node.setCost(calcNodeCost(node, target_heading_r));

                                neighbors.emplace_back(std::move(node));
                            });                        
                    }
                });
        });
    
    return neighbors;
}

std::vector<float64_t> AStar::calcPossibleLongitudinalVelocitiesMps(const GraphNode& current_node) const noexcept
{ 
    const float64_t   time_step_s   = m_cfg->getTimeStepMs()/1000.0;
    const float64_t   start_vel_mps = current_node.getEstimatedLongitudinalVelocityMps();    
    const float64_t   max_vel_mps   = std::min<float64_t>(start_vel_mps + m_cfg->getMaxLongitudinalAccelMpss()*time_step_s,  m_cfg->getMaxVelMps());
    const float64_t   min_vel_mps   = std::max<float64_t>(start_vel_mps - m_cfg->getMaxLongitudinalAccelMpss()*time_step_s, -m_cfg->getMaxVelMps());
    const float64_t   vel_diff_mps  = std::fabs(max_vel_mps - min_vel_mps);
    const std::size_t num_pos_vels  = static_cast<std::size_t>(std::floor(vel_diff_mps/m_cfg->getVelocityDiscretizationMps()));

    std::vector<float64_t> possible_vels_mps;
    possible_vels_mps.reserve(num_pos_vels);
    for(std::size_t vel_it = 0U; vel_it < num_pos_vels; ++vel_it)
    {
        possible_vels_mps.emplace_back(min_vel_mps + static_cast<float64_t>(vel_it)/static_cast<float64_t>(num_pos_vels)*vel_diff_mps);
    }

    return possible_vels_mps;
}

std::vector<float64_t> AStar::calcPossibleLateralVelocitiesMps(const GraphNode& current_node) const noexcept
{
    const float64_t   time_step_s  = m_cfg->getTimeStepMs()/1000.0;
    const float64_t   start_vel_mps = current_node.getEstimatedLateralVelocityMps();    
    const float64_t   max_vel_mps   = std::min<float64_t>(start_vel_mps + m_cfg->getMaxLateralAccelMpss()*time_step_s,  m_cfg->getMaxVelMps());
    const float64_t   min_vel_mps   = std::max<float64_t>(start_vel_mps - m_cfg->getMaxLateralAccelMpss()*time_step_s, -m_cfg->getMaxVelMps());
    const float64_t   vel_diff_mps  = std::fabs(max_vel_mps - min_vel_mps);
    const std::size_t num_pos_vels  = static_cast<std::size_t>(std::floor(vel_diff_mps/m_cfg->getVelocityDiscretizationMps()));

    std::vector<float64_t> possible_vels_mps;
    possible_vels_mps.reserve(num_pos_vels);
    for(std::size_t vel_it = 0U; vel_it < num_pos_vels; ++vel_it)
    {
        possible_vels_mps.emplace_back(min_vel_mps + static_cast<float64_t>(vel_it)/static_cast<float64_t>(num_pos_vels)*vel_diff_mps);
    }

    return possible_vels_mps;
}

std::vector<float64_t> AStar::calcPossibleYawRatesRps(const GraphNode& current_node) const noexcept
{
    const float64_t   time_step_s        = m_cfg->getTimeStepMs()/1000.0;
    const float64_t   start_yaw_rate_rps = current_node.getEstimatedHeadingR();    
    const float64_t   max_yaw_rate_rps   = std::min<float64_t>(start_yaw_rate_rps + m_cfg->getMaxYawRateRateRpss()*time_step_s,  m_cfg->getMaxYawRateRps());
    const float64_t   min_yaw_rate_rps   = std::min<float64_t>(start_yaw_rate_rps - m_cfg->getMaxYawRateRateRpss()*time_step_s, -m_cfg->getMaxYawRateRps());
    const float64_t   yaw_rate_diff_rps  = std::fabs(max_yaw_rate_rps - min_yaw_rate_rps);
    const std::size_t num_pos_yaw_rates  = static_cast<std::size_t>(std::floor(yaw_rate_diff_rps/m_cfg->getYawRateDiscretizationRps()));

    std::vector<float64_t> possible_yaw_rates_rps;
    possible_yaw_rates_rps.reserve(num_pos_yaw_rates);
    for(std::size_t yaw_it = 0U; yaw_it < num_pos_yaw_rates; ++yaw_it)
    {
        possible_yaw_rates_rps.emplace_back(min_yaw_rate_rps + static_cast<float64_t>(yaw_it)/static_cast<float64_t>(num_pos_yaw_rates)*yaw_rate_diff_rps);
    }

    return possible_yaw_rates_rps;
}

float64_t AStar::calcNodeGScore(const GraphNode& parent_node, const GraphNode& node) const noexcept
{
    return (parent_node.getG() + std::sqrt(std::pow(parent_node.getEstimatedPointM().getX() - node.getEstimatedPointM().getX(), 2U) +
                                           std::pow(parent_node.getEstimatedPointM().getY() - node.getEstimatedPointM().getY(), 2U)));
}

float64_t AStar::calcNodeCost(const GraphNode& node, const float64_t target_heading_r) const
{
    return (node.getG() + calcHeuristic(node, target_heading_r));
}

float64_t AStar::calcHeuristic(const GraphNode& node, const float64_t target_heading_r) const
{
    const float64_t max_dist_m        = std::sqrt(std::pow(m_goal_node.getEstimatedPointM().getX() - m_data.getLocalPose()->pose.pose.position.x, 2U) +
                                                  std::pow(m_goal_node.getEstimatedPointM().getY() - m_data.getLocalPose()->pose.pose.position.y, 2U));
    const float64_t dist_to_goal_m    = std::sqrt(std::pow(m_goal_node.getEstimatedPointM().getX() - node.getEstimatedPointM().getX(), 2U) +
                                                  std::pow(m_goal_node.getEstimatedPointM().getY() - node.getEstimatedPointM().getY(), 2U));
    const float64_t dist_heuristic    = dist_to_goal_m / max_dist_m*m_cfg->getHeuristicConfig().getDistWeight();

    const float64_t heading_diff      = std::fabs(target_heading_r - node.getEstimatedHeadingR());
    const float64_t heading_heuristic = heading_diff/M_PI*m_cfg->getHeuristicConfig().getHeadingWeight();


    const float64_t cur_velocity_mps  = std::sqrt(std::pow(node.getEstimatedLateralVelocityMps(), 2U) +
                                                  std::pow(node.getEstimatedLongitudinalVelocityMps(), 2U));
    const float64_t max_velocity_mps = m_cfg->getMaxVelMps();                                                  
    const float64_t goal_velocity_mps = std::sqrt(std::pow(m_goal_node.getEstimatedLateralVelocityMps(), 2U) + 
                                                  std::pow(m_goal_node.getEstimatedLongitudinalVelocityMps(), 2U));

    float64_t speed_heuristic = 1.0 - cur_velocity_mps / max_velocity_mps;
    if(nodeNeedsToSlow(node, node.getEstimatedHeadingR(), cur_velocity_mps, goal_velocity_mps) == true)
    {
        speed_heuristic = std::fabs(cur_velocity_mps - goal_velocity_mps) / (goal_velocity_mps - max_velocity_mps);
    }

    speed_heuristic *= m_cfg->getHeuristicConfig().getSpeedWeight();

    return std::fabs(dist_heuristic + heading_heuristic + speed_heuristic);
}

bool AStar::nodeNeedsToSlow(const GraphNode& current_node, const float64_t heading_r, const float64_t velocity_mps, const float64_t goal_velocity_mps) const
{
    float64_t max_accel_mpss = std::sqrt(std::pow(m_cfg->getMaxLongitudinalAccelMpss(), 2U) + std::pow(m_cfg->getMaxLateralAccelMpss(), 2U));
    float64_t next_vel_mps = velocity_mps + max_accel_mpss*m_cfg->getTimeStepMs()/1000.0;
    if (next_vel_mps > m_cfg->getMaxVelMps())
    {
        next_vel_mps = m_cfg->getMaxVelMps();
    }

     const float64_t average_vel_mps = (next_vel_mps + velocity_mps)/2.0;
     const float64_t x_m = current_node.getEstimatedPointM().getX() + average_vel_mps*std::cos(heading_r);
     const float64_t y_m = current_node.getEstimatedPointM().getY() + average_vel_mps*std::sin(heading_r);
     const float64_t dx_m = x_m - m_goal_node.getEstimatedPointM().getX();
     const float64_t dy_m = y_m - m_goal_node.getEstimatedPointM().getY();
     const float64_t dist_to_goal_m = std::sqrt(std::pow(dx_m, 2U) + std::pow(dy_m, 2U));
     const float64_t time_to_decel_s = (velocity_mps - goal_velocity_mps) / max_accel_mpss;
     const float64_t dist_to_decel_m = velocity_mps * time_to_decel_s - max_accel_mpss*std::pow(time_to_decel_s, 2U)/2.0;

     return (dist_to_goal_m <= dist_to_decel_m);
}

}// namespace local_planner
