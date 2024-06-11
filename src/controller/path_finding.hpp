#pragma once

#include <iostream>
#include <tuple>
#include "labyrinth.hpp"
#include "path.hpp"

bool contains( const std::vector<PathNode> &visitedNodes, const PathNode &element );
bool isInsideMaze( int x, int y, size_t labyrinth_size );
bool isDirectNeighbor( size_t dx, size_t dy );

/**
 * @brief Finds the shortest path from the start position to the target position in the labyrinth.
 *
 * This function uses the breadth-first search (BFS) algorithm to find the shortest path
 * from the specified start position to the target position within the given labyrinth.
 *
 * @param labyrinth The labyrinth in which the robot must navigate.
 * @param startPosition The starting position of the robot.
 * @param targetPosition The target position the robot must reach.
 * @return The shortest path from the start position to the target position.
 *         If no path exists, an empty path is returned.
 */
template <size_t S>
Path findPath(const Labyrinth<S> &labyrinth, const std::tuple<size_t, size_t> &startPosition,
              const std::tuple<size_t, size_t> &targetPosition)
              {
    if ( startPosition == targetPosition )
        return {};

    std::vector<PathNode> visitedNodes;

    // Initializing the queue for BFS
    std::deque<std::tuple<PathNode, Path>> nodeDeque;

    // define the start and end node
    PathNode startNode{};
    startNode.x = std::get<0>( startPosition );
    startNode.y = std::get<1>( startPosition );
    visitedNodes.emplace_back( startNode );

    PathNode targetNode{};
    targetNode.x = std::get<0>( targetPosition );
    targetNode.y = std::get<1>( targetPosition );

    // Initialize the path with the start node
    Path startPath = {};
    startPath.pushNode( startNode.x, startNode.y );
    nodeDeque.emplace_back( startNode, startPath );

    while ( !nodeDeque.empty())
    {
        // get the first node of the deque
        auto &currentTuple = nodeDeque.front();
        PathNode node = std::get<0>( currentTuple );
        Path path = std::get<1>( currentTuple );
        nodeDeque.pop_front();

        // if current node is the target node
        if ( node == targetNode )
        {
            return path;
        }

        // add neighbor fields of type AIR to the queue
        for ( int dx = -1; dx <= 1; dx++ )
        {
            for ( int dy = -1; dy <= 1; dy++ )
            {
                // if neighbor is air && inside the maze
                const int newX = static_cast<int>(node.x) + dx;
                const int newY = static_cast<int>(node.y) + dy;
                if ( isInsideMaze( newX, newY, S ) && isDirectNeighbor( dx, dy ) && !labyrinth.get( newX, newY ))
                {
                    PathNode nodeToFind{};
                    nodeToFind.x = newX;
                    nodeToFind.y = newY;
                    // node was not visited before -> add node to queue
                    if ( !contains( visitedNodes, nodeToFind ))
                    {
                        Path updatedPath = path;
                        updatedPath.pushNode( nodeToFind.x, nodeToFind.y );
                        nodeDeque.emplace_back( nodeToFind, updatedPath );
                        visitedNodes.emplace_back( nodeToFind );
                    }
                }
            }
        }
    }
    std::cerr << "There is no possible path for this maze." << std::endl;
    return {};
}

