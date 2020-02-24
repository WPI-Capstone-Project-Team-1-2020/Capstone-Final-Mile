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
    start_node.setEstimatedVelocityMps(std::sqrt(std::pow(m_data.getLocalPose()->twist.twist.linear.x, 2U) + 
                                                 std::pow(m_data.getLocalPose()->twist.twist.linear.y, 2U)));
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

}

std::vector<float64_t> AStar::calcPossibleVelocitiesMps(const GraphNode& current_node)
{
    const float64_t start_vel_mps = current_node.getEstimatedVelocityMps();
    const float64_t time_step_ms  = m_cfg->getTimeStepMs();
}

}// namespace local_planner