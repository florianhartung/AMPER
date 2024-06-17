#pragma once

#include <iostream>
#include <algorithm>
#include <array>
#include <queue>


struct PathNode
{
    size_t x;
    size_t y;

    bool operator==( const PathNode &other ) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=( const PathNode &other ) const
    {
        return !(*this == other);
    }

    // a* will use this operator in the priority queue if the priorities
    // are the same for two elements. Comparison would not make sense in
    // any other case
    bool operator<( const PathNode &other ) const
    {
        return  x + y < other.x + other.y;
    }

    bool operator>( const PathNode &other ) const
    {
        return other < *this;
    }
};


/**
 * @brief basically a FIFO queue holding the path
*/
class Path
{
public:
    Path() = default;

    // the node is emplaced at back of the path
    void pushNode( size_t x, size_t y );
    // the first node will be removed and returned
    PathNode popNode();

    size_t size();

    friend std::ostream &operator<<( std::ostream &os, const Path &path );
    void reverse();

private:
    std::deque<PathNode> m_nodes{};

};


