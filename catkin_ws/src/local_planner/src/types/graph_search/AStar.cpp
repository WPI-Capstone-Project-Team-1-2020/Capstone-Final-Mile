// Component
#include "AStar.hpp"
#include "LocalPlannerConfig.hpp"
#include "RosConversionHelper.hpp"

// Ros
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// Standard
#include <utility>


namespace local_planner
{

std::uint64_t GraphNode::id_generator{0};
GraphNodeToleranceConfig GraphNode::tolerance_cfg{};

AStar::AStar(const std::shared_ptr<LocalPlannerConfig>& cfg) :
    m_cfg{cfg}
{
    GraphNode::tolerance_cfg = m_cfg->getGraphNodeToleranceConfig();
}

AStar::~AStar(){}

bool AStar::update()
{
    resetPlanner();
    initializePlanner();
    planTrajectory();
}

void AStar::resetPlanner() noexcept
{
   m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>>();
   m_open_nodes.clear(); 
   m_closed_nodes.clear();
   GraphNode::id_generator = 0U;
}

void AStar::initializePlanner() noexcept
{
    GraphNode start_node;    
    start_node.setEstimatedPointM(Point(m_data.getLocalPose()->pose.pose.position.x, m_data.getLocalPose()->pose.pose.position.y));
    start_node.setEstimatedLongitudinalVelocityMps(m_data.getLocalPose()->twist.twist.linear.x);
    start_node.setEstimatedLateralVelocityMps(m_data.getLocalPose()->twist.twist.linear.y);
    start_node.setEstimatedHeadingR(RosConversionHelper::quaternionMsgToYawR(m_data.getLocalPose()->pose.pose.orientation));
    start_node.setEstimatedYawRateRps(m_data.getLocalPose()->twist.twist.angular.z);    
    start_node.setCost(std::numeric_limits<float64_t>::max());
    m_frontier.emplace(start_node);
    m_open_nodes.emplace(std::move(start_node));
}

void AStar::planTrajectory()
{    
    while (true)
    {
        if (m_frontier.empty() == false)
        {
            GraphNode current_node = m_frontier.top();
            m_frontier.pop();
            expandFrontier(current_node);
        }
        else
        {
            ROS_ERROR_STREAM("Frontier empty, all possible nodes explored");
            return;
        }
    }
}

void AStar::expandFrontier(const GraphNode& current_node)
{
    std::vector<GraphNode> neighbors = calcNeighbors(current_node);
    std::for_each(std::make_move_iterator(neighbors.begin()),
        std::make_move_iterator(neighbors.end()),
        [&frontier = this->m_frontier, &open_nodes = this->m_open_nodes](GraphNode&& node) -> void
        {            
            frontier.emplace(node);
            open_nodes.emplace(std::move(node));
        });
}

std::vector<GraphNode> AStar::calcNeighbors(const GraphNode& current_node)
{
    std::vector<float64_t> possible_lon_velocities_mps = calcPossibleLongitudinalVelocitiesMps(current_node);
    std::vector<float64_t> possible_lat_velocities_mps = calcPossibleLateralVelocitiesMps(current_node);
    std::vector<float64_t> possible_yaw_rates_rps      = calcPossibleYawRatesRps(current_node);

    std::vector<GraphNode> neighbors;
    neighbors.reserve(possible_lon_velocities_mps.size()*possible_lat_velocities_mps.size()*possible_yaw_rates_rps.size());

    std::for_each(possible_lon_velocities_mps.cbegin(),
        possible_lon_velocities_mps.cend(),
        [&neighbors, &possible_lat_velocities_mps, &possible_yaw_rates_rps, &current_node, this](const float64_t x_vel_mps) -> void
        {
            std::for_each(possible_lat_velocities_mps.cbegin(),
                possible_lat_velocities_mps.cend(),
                [&neighbors, x_vel_mps, &possible_yaw_rates_rps, &current_node, this](const float64_t y_vel_mps) -> void
                {                    
                    if (std::sqrt(std::pow(x_vel_mps, 2U) + std::pow(y_vel_mps, 2U)) <= m_cfg->getMaxVelMps())
                    {
                        std::for_each(possible_yaw_rates_rps.cbegin(),
                            possible_yaw_rates_rps.cend(),
                            [&neighbors, x_vel_mps, y_vel_mps, &current_node, this](const float64_t yaw_rate_rps) -> void
                            {
                                GraphNode node;
                                
                                node.setParentID(current_node.getID());

                                node.setCommandedLongitudinalVelocityMps(x_vel_mps);
                                node.setCommandedLateralVelocityMps(y_vel_mps);
                                node.setCommandedYawRateRps(yaw_rate_rps);

                                /// @TODO Add controller transient response estimation
                                node.setEstimatedLongitudinalVelocityMps(x_vel_mps);
                                node.setEstimatedLateralVelocityMps(y_vel_mps);
                                node.setEstimatedYawRateRps(yaw_rate_rps);

                                const float64_t time_step_s = m_cfg->getTimeStepMs()/1000.0;
                                const float64_t x_m         = current_node.getEstimatedPointM().getX() + 
                                                              x_vel_mps*time_step_s + 
                                                              m_cfg->getMaxLongitudinalAccelMpss()*std::pow(time_step_s, 2U)/2.0;
                                const float64_t y_m         = current_node.getEstimatedPointM().getY() + 
                                                              y_vel_mps*time_step_s + 
                                                              m_cfg->getMaxLateralAccelMpss()*std::pow(time_step_s, 2U)/2.0;
                                const float64_t heading_r   = current_node.getEstimatedHeadingR() + 
                                                              yaw_rate_rps*time_step_s + 
                                                              m_cfg->getMaxYawRateRateRpss()*std::pow(time_step_s, 2U)/2.0;  

                                node.setEstimatedPointM(Point(x_m, y_m));
                                node.setEstimatedHeadingR(heading_r);

                                node.setG(calcNodeGScore(current_node, node));
                                node.setCost(calcNodeCost(node));

                                neighbors.emplace_back(std::move(node));
                            });                        
                    }
                });
        });

}

std::vector<float64_t> AStar::calcPossibleLongitudinalVelocitiesMps(const GraphNode& current_node) const noexcept
{ 
    const float64_t   time_step_s   = m_cfg->getTimeStepMs()/1000.0;
    const float64_t   start_vel_mps = current_node.getEstimatedLongitudinalVelocityMps();    
    const float64_t   max_vel_mps   = std::min<float64_t>(start_vel_mps + m_cfg->getMaxLongitudinalAccelMpss()*time_step_s,  m_cfg->getMaxVelMps());
    const float64_t   min_vel_mps   = std::max<float64_t>(start_vel_mps - m_cfg->getMaxLongitudinalAccelMpss()*time_step_s, -m_cfg->getMaxVelMps());
    const float64_t   vel_diff_mps  = std::fabs(max_vel_mps - min_vel_mps);
    const std::size_t num_pos_vels  = static_cast<std::size_t>(std::floor(vel_diff_mps/m_cfg->getMaxLongitudinalAccelMpss()));

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
    const float64_t   time_step_ms  = m_cfg->getTimeStepMs();
    const float64_t   start_vel_mps = current_node.getEstimatedLateralVelocityMps();    
    const float64_t   max_vel_mps   = std::min<float64_t>(start_vel_mps + m_cfg->getMaxLateralAccelMpss()*time_step_ms,  m_cfg->getMaxVelMps());
    const float64_t   min_vel_mps   = std::min<float64_t>(start_vel_mps - m_cfg->getMaxLateralAccelMpss()*time_step_ms, -m_cfg->getMaxVelMps());
    const float64_t   vel_diff_mps  = std::fabs(max_vel_mps - min_vel_mps);
    const std::size_t num_pos_vels  = static_cast<std::size_t>(std::floor(vel_diff_mps/m_cfg->getMaxLateralAccelMpss()));

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
    const float64_t   time_step_ms       = m_cfg->getTimeStepMs();
    const float64_t   start_yaw_rate_rps = current_node.getEstimatedHeadingR();    
    const float64_t   max_yaw_rate_rps   = std::min<float64_t>(start_yaw_rate_rps + m_cfg->getMaxYawRateRateRpss()*time_step_ms,  m_cfg->getMaxYawRateRps());
    const float64_t   min_yaw_rate_rps   = std::min<float64_t>(start_yaw_rate_rps - m_cfg->getMaxYawRateRateRpss()*time_step_ms, -m_cfg->getMaxYawRateRps());
    const float64_t   yaw_rate_diff_rps  = std::fabs(max_yaw_rate_rps - min_yaw_rate_rps);
    const std::size_t num_pos_yaw_rates  = static_cast<std::size_t>(std::floor(yaw_rate_diff_rps/m_cfg->getMaxYawRateRateRpss()));

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

float64_t AStar::calcNodeCost(const GraphNode& node) const noexcept
{
    const float64_t dist_to_goal_m = std::sqrt(std::pow(m_data.getGoalPose()->x - node.getEstimatedPointM().getX(), 2U) +
                                     std::pow(m_data.getGoalPose()->y - node.getEstimatedPointM().getY(), 2U));
}

}// namespace local_planner