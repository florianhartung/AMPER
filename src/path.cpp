#include "path.hpp"

void Path::pushNode(size_t x, size_t y) 
{
    PathNode node; 
    node.x = x; 
    node.y = y; 
    m_nodes.push_back(node);
}

PathNode Path::popNode() 
{ 
    PathNode node = m_nodes[0];
    m_nodes.pop_front();  
    return node;
}
