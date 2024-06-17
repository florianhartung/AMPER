#include "path.hpp"

void Path::pushNode( size_t x, size_t y )
{
    PathNode node;
    node.x = x;
    node.y = y;
    m_nodes.push_back( node );
}

PathNode Path::popNode()
{
    PathNode node = m_nodes[0];
    m_nodes.pop_front();
    return node;
}

size_t Path::size()
{
    return m_nodes.size();
}


void Path::reverse()
{
    std::reverse( m_nodes.begin(), m_nodes.end());
}


std::ostream &operator<<( std::ostream &os, const Path &path )
{
    os << "Path: ";
    for ( const auto &node: path.m_nodes )
    {
        os << "(" << node.x << ", " << node.y << ") ";
    }
    return os;
}