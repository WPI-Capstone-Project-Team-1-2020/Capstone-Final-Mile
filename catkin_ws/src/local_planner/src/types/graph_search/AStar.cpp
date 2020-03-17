// Component
#include "AStar.hpp"
#include "ForwardSimHelper.hpp"
#include "LocalPlannerConfig.hpp"
#include "RosConversionHelper.hpp"

// Standard
#include <utility>

namespace local_planner
{

std::uint64_t GraphNode::id_generator{0U};
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
    return planTrajectory();
}

void AStar::resetPlanner() noexcept
{
   m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>>();
   m_open_nodes.clear(); 
   m_nodes.clear();
   m_goal_node = GraphNode();
   GraphNode::id_generator = 0U;
}

void AStar::initializePlanner() noexcept
{
    GraphNode start_node;    
    start_node.setEstimatedPointM(Point(m_data.getLocalPose()->pose.pose.position.x, m_data.getLocalPose()->pose.pose.position.y));
    start_node.setCost(std::numeric_limits<float64_t>::max());

    m_frontier.emplace(start_node);
    m_open_nodes.emplace(start_node);
    m_nodes.emplace(start_node.getID(), std::move(start_node));

    m_goal_node.setEstimatedPointM(Point(m_data.getGoalPose()->x_m, m_data.getGoalPose()->y_m));
}

bool AStar::planTrajectory()
{  
    while (true)
    {
        if (m_frontier.empty() == false)
        {
            GraphNode current_node = m_frontier.top();

            if (current_node == m_goal_node)
            {
                m_goal_node.setParentID(current_node.getParentID());
                
                if (reconstructPath() == true)
                {
                    return true;
                }

                ROS_ERROR_STREAM("Unable to reconstruct path");

                return false;
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
    
    std::for_each(std::make_move_iterator(neighbors.begin()),
        std::make_move_iterator(neighbors.end()),
        [&current_node, &frontier = this->m_frontier, &open_nodes = this->m_open_nodes, &nodes = this->m_nodes](GraphNode&& node) -> void
        {
            nodes.emplace(node.getID(), node);

            std::unordered_set<GraphNode>::const_iterator open_it = open_nodes.find(node);

            if (open_it != open_nodes.cend())
            {
                if(node.getCost() >= open_it->getCost())
                {
                    return;
                }
            }                                    
            else
            {
                frontier.emplace(node);
                open_nodes.emplace(std::move(node));
            }
        });
}

std::vector<GraphNode> AStar::calcNeighbors(const GraphNode& current_node)
{    
    std::vector<GraphNode> neighbors;
    neighbors.reserve(8U);

    GraphNode neighbor_1;  
    GraphNode neighbor_2;
    GraphNode neighbor_3;
    GraphNode neighbor_4;
    GraphNode neighbor_5;
    GraphNode neighbor_6;
    GraphNode neighbor_7;
    GraphNode neighbor_8;
    
    neighbor_1.setEstimatedPointM(Point(current_node.getPointM().getX() + m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY()));
    neighbor_2.setEstimatedPointM(Point(current_node.getPointM().getX() - m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY()));
    neighbor_3.setEstimatedPointM(Point(current_node.getPointM().getX() + m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY() - m_cfg->getGraphNodeToleranceConfig().getToleranceM()));                                        
    neighbor_4.setEstimatedPointM(Point(current_node.getPointM().getX() + m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY() + m_cfg->getGraphNodeToleranceConfig().getToleranceM()));
    neighbor_5.setEstimatedPointM(Point(current_node.getPointM().getX() - m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY() + m_cfg->getGraphNodeToleranceConfig().getToleranceM()));
    neighbor_6.setEstimatedPointM(Point(current_node.getPointM().getX() - m_cfg->getGraphNodeToleranceConfig().getToleranceM(), 
                                        current_node.getPointM().getY() - m_cfg->getGraphNodeToleranceConfig().getToleranceM()));
    neighbor_7.setEstimatedPointM(Point(current_node.getPointM().getX(), 
                                        current_node.getPointM().getY() + m_cfg->getGraphNodeToleranceConfig().getToleranceM()));                                                                                                                                                                
    neighbor_8.setEstimatedPointM(Point(current_node.getPointM().getX(), 
                                        current_node.getPointM().getY() - m_cfg->getGraphNodeToleranceConfig().getToleranceM()));                                        

    neighbors.emplace_back(neighbor_1);
    neighbors.emplace_back(neighbor_2);
    neighbors.emplace_back(neighbor_3);
    neighbors.emplace_back(neighbor_4);
    neighbors.emplace_back(neighbor_5);
    neighbors.emplace_back(neighbor_6);
    neighbors.emplace_back(neighbor_7);
    neighbors.emplace_back(neighbor_8);

    std::for_each(neighbors.begin(),
        neighbors.end(),
        [&current_node, this](GraphNode& node) -> void
        {
            node.setParentID(current_node.getID());
            node.setG(calcNodeGScore(current_node, node));
            node.setCost(calcNodeCost(node));
        });                                        

    return neighbors;
}

float64_t AStar::calcNodeGScore(const GraphNode& parent_node, const GraphNode& node) const noexcept
{
    return (parent_node.getG() + std::sqrt(std::pow(parent_node.getPointM().getX() - node.getPointM().getX(), 2U) +
                                           std::pow(parent_node.getPointM().getY() - node.getPointM().getY(), 2U)));
}

float64_t AStar::calcNodeCost(const GraphNode& node) const
{
    return (node.getG() + std::sqrt(std::pow(node.getPointM().getX() - m_goal_node.getPointM().getX(), 2U) +
                                    std::pow(node.getPointM().getY() - m_goal_node.getPointM().getY(), 2U)));
}

bool AStar::reconstructPath()
{
    m_path.clear();
    GraphNode cur_node = m_goal_node;
    m_path.emplace_back(cur_node.getPointM());
    constexpr std::uint64_t start_id{0U};

    while (cur_node.getID() != start_id)
    {                 
        std::unordered_map<std::uint64_t, GraphNode>::const_iterator parent_it = m_nodes.find(cur_node.getParentID());
        
        if (parent_it != m_nodes.cend())
        {
            cur_node = parent_it->second;
        }
        else
        {
            return false;
        }

        m_path.emplace_back(cur_node.getPointM());
    }

    std::reverse(m_path.begin(), m_path.end());

    return true;
}

}// namespace local_planner
