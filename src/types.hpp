#pragma once

#include <iostream> 
#include <array>
#include <queue> 

constexpr size_t LABYRINTH_SIZE = 4;

/**
 * @brief datatype for the labyrinth
 * holds an array of bools (true: block, false: air)
 * the array holds 2d map, column first
*/
class Labyrinth {
public: 
    Labyrinth(const std::array<bool, LABYRINTH_SIZE * LABYRINTH_SIZE> &labyrinth);

    bool get(size_t x, size_t y) const; 

private: 
    std::array<bool, LABYRINTH_SIZE * LABYRINTH_SIZE> m_cells = {}; 
}; 


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


