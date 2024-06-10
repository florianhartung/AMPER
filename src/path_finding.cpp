#include <algorithm>
#include <vector>
#include "path_finding.hpp"


bool contains( const std::vector<PathNode> &visitedNodes, const PathNode &element )
{
    return std::find( visitedNodes.begin(), visitedNodes.end(), element ) != visitedNodes.end();
}


bool isInsideMaze( int x, int y )
{
    return (x >= 0 && x < LABYRINTH_SIZE && y >= 0 && y < LABYRINTH_SIZE);
}


bool isDirectNeighbor( size_t dx, size_t dy )
{
    return ((dx == 0 || dy == 0) && dx != dy);
}


Path findPath( const Labyrinth &labyrinth, const std::tuple<size_t, size_t> &startPosition,
               const std::tuple<size_t, size_t> &targetPosition )
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
                if ( isInsideMaze( newX, newY ) && isDirectNeighbor( dx, dy ) && labyrinth.get( newX, newY ))
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
