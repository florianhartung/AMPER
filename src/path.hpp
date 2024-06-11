#pragma once

#include <iostream> 
#include <array>
#include <queue> 


struct PathNode {
    size_t x; 
    size_t y; 

    bool operator==(const PathNode &other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief basically a FIFO queue holding the path
*/
class Path {
public: 
    Path() = default; 

    // the node is emplaced at back of the path
    void pushNode(size_t x, size_t y);
    // the first node will be removed and returned 
    PathNode popNode(); 

private: 
    std::deque<PathNode> m_nodes {}; 

};

