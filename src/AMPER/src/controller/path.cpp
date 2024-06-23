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


std::unique_ptr<PathNode> Path::peekNode() {
    if (m_nodes.size() == 0) {
        return nullptr;
    }

    return std::make_unique<PathNode>(std::move(m_nodes.front()));
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