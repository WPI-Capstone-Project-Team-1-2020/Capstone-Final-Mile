// Component
#include "AStar.hpp"
#include "LocalPlannerConfig.hpp"


namespace local_planner
{

AStar::AStar(const std::shared_ptr<LocalPlannerConfig>& cfg) :
    m_cfg{cfg}
{

}

AStar::~AStar(){}

bool AStar::update()
{
    
}

void AStar::resetPlanner() noexcept
{
   m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>>();
   m_open_nodes.clear(); 
   m_closed_nodes.clear();
}

}// namespace local_planner