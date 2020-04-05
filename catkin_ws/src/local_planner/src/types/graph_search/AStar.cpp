// Component
#include "AStar.hpp"
#include "ForwardSimHelper.hpp"
#include "LocalPlannerConfig.hpp"
#include "RosConversionHelper.hpp"

// Libraries
#include <chrono>
#include <unsupported/Eigen/Splines>

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

AStar::~AStar() = default;

bool AStar::update()
{
    resetPlanner();
    
    if (initializePlanner() == false)
    {
        ROS_ERROR_STREAM("Failed to initialize planner");

        return false;
    }

    return planTrajectory();
}

void AStar::resetPlanner() noexcept
{
   m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>>();
   m_open_nodes.clear(); 
   m_nodes.clear();
   m_path.clear();
   m_ros_path = nav_msgs::Path();
   m_goal_node = GraphNode();
   GraphNode::id_generator = 0U;
}

bool AStar::initializePlanner() noexcept
{
    m_costmap.setOccupancyGrid(m_data.getCostmap(), m_data.getLocalPose()->pose.pose);

    GraphNode start_node;    
    start_node.setEstimatedPointM(Point(0.0, 0.0));
    start_node.setCost(std::numeric_limits<float64_t>::max());

    m_frontier.emplace(start_node);
    m_open_nodes.emplace(start_node);
    m_nodes.emplace(start_node.getID(), std::move(start_node));

    const float64_t dx_m      = m_data.getGoalPose()->x_m - m_data.getLocalPose()->pose.pose.position.x;
    const float64_t dy_m      = m_data.getGoalPose()->y_m - m_data.getLocalPose()->pose.pose.position.y;    
    const float64_t dp_m      = std::sqrt(std::pow(dx_m, 2U) + std::pow(dy_m, 2U));
    
    if (dp_m > 40.0)
    {        
        return false;
    }
    
    const float64_t goal_x_m  = dp_m;
    const float64_t goal_y_m  = 0.0;

    m_goal_node.setEstimatedPointM(Point(goal_x_m, goal_y_m));

    return true;
}

bool AStar::planTrajectory()
{  
    const auto start_time_ns = std::chrono::high_resolution_clock::now();

    while (true)
    {
        const auto current_time_ns = std::chrono::high_resolution_clock::now();
        const auto dt_ms           = std::chrono::duration_cast<std::chrono::milliseconds>(current_time_ns - start_time_ns);

        if (dt_ms.count() > m_cfg->getSearchTimeoutMs())
        {
            ROS_ERROR_STREAM("Failed to plan path during given time allotted");

            return false;
        }

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
        [&current_node, this](GraphNode&& node) -> void
        {
            const std::int64_t prob_coll = getProbabilityCollision(node.getPointM());

            if (prob_coll >= m_cfg->getLethalProbability())
            {
                return;
            }
            
            node.setCost(node.getCost() + prob_coll*m_cfg->getNonLethalCostMult());
            m_nodes.emplace(node.getID(), node);

            std::unordered_set<GraphNode>::const_iterator open_it = m_open_nodes.find(node);

            if (open_it != m_open_nodes.cend())
            {
                if(node.getCost() >= open_it->getCost())
                {
                    return;
                }
            }                                    
            else
            {
                m_frontier.emplace(node);
                m_open_nodes.emplace(std::move(node));
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

std::int64_t AStar::getProbabilityCollision(const Point& pt)
{
    const float64_t dx_m      = m_data.getGoalPose()->x_m - m_data.getLocalPose()->pose.pose.position.x;
    const float64_t dy_m      = m_data.getGoalPose()->y_m - m_data.getLocalPose()->pose.pose.position.y;    
    const float64_t heading_r = std::atan2(dy_m, dx_m);
    const float64_t dp_m      = std::sqrt(std::pow(pt.getX(), 2U) + std::pow(pt.getY(), 2U));
    const float64_t x_m       = m_data.getLocalPose()->pose.pose.position.x + dp_m*std::cos(heading_r);
    const float64_t y_m       = m_data.getLocalPose()->pose.pose.position.y + dp_m*std::sin(heading_r);        

    return m_costmap.getProbabilityOccupied(Point(x_m, y_m));
}

bool AStar::reconstructPath()
{
    auto transform_pt = [&data = this->m_data](const Point& pt) -> Point
    {

        const float64_t dx_m      = data.getGoalPose()->x_m - data.getLocalPose()->pose.pose.position.x;
        const float64_t dy_m      = data.getGoalPose()->y_m - data.getLocalPose()->pose.pose.position.y;    
        const float64_t heading_r = std::atan2(dy_m, dx_m);
        const float64_t dp_m      = std::sqrt(std::pow(pt.getX(), 2U) + std::pow(pt.getY(), 2U));
        const float64_t x_m       = data.getLocalPose()->pose.pose.position.x + dp_m*std::cos(heading_r);
        const float64_t y_m       = data.getLocalPose()->pose.pose.position.y + dp_m*std::sin(heading_r);        

        return Point(x_m, y_m);
    };

    m_path.clear();
    GraphNode cur_node = m_goal_node;
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

        m_path.emplace_back(transform_pt(cur_node.getPointM()));
    }

    std::reverse(m_path.begin(), m_path.end());

    using Spline1d               = Eigen::Spline<float64_t, 2>;
    using Spline1dFitting        = Eigen::SplineFitting<Spline1d>;
    using KnotVectorType         = Spline1d::KnotVectorType;
    using ControlPointVectorType = Spline1d::ControlPointVectorType;

    ControlPointVectorType ctrl_pts(2, m_path.size());    
    KnotVectorType         chord_lengths;

    for (std::size_t pt_it = 0U; pt_it < m_path.size(); ++pt_it)
    {   
        ctrl_pts(0, pt_it) = m_path[pt_it].getX();
        ctrl_pts(1, pt_it) = m_path[pt_it].getY();
    }

    Eigen::ChordLengths(ctrl_pts, chord_lengths);

    const Spline1d spline = Spline1dFitting::Interpolate(ctrl_pts, 3, chord_lengths);    

    std::vector<Point> smoothed_path;
    smoothed_path.reserve(m_path.size());
    const std::size_t num_spline_pts = m_path.size();

    for (std::size_t spline_it = 0U; spline_it < num_spline_pts; ++spline_it)
    {        
        const float64_t spline_disc_pt  = static_cast<float64_t>(spline_it)/static_cast<float64_t>(num_spline_pts);
        const Eigen::MatrixXd spline_pt = spline.operator()(spline_disc_pt);
        Point tmp_pt(spline_pt(0), spline_pt(1));
        smoothed_path.emplace_back(std::move(tmp_pt));
    }

    m_path = std::move(smoothed_path);

    m_ros_path.poses.reserve(m_path.size());
    std::for_each(m_path.cbegin(),
        m_path.cend(),
        [&ros_path = this->m_ros_path, height_m = m_data.getLocalPose()->pose.pose.position.z](const Point& pt)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp    = ros::Time::now();
            pose.pose.position.x = pt.getX();
            pose.pose.position.y = pt.getY();
            pose.pose.position.z = height_m;

            ros_path.poses.emplace_back(pose);
        });

        m_ros_path.header.frame_id = "world";
        m_ros_path.header.stamp    = ros::Time::now();

    return true;
}

}// namespace local_planner
