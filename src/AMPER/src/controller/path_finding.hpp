#pragma once

#include <unordered_map>
#include "labyrinth.hpp"
#include "path.hpp"


bool isInsideMaze( int x, int y, size_t labyrinth_size );
int heuristic( const PathNode &a, const PathNode &b );


// custom priority queue in order to use pairs instead of single value
template<typename T, typename priority_t>
struct PairPriorityQueue
{
    typedef std::pair<priority_t, T> element;
    std::priority_queue<element, std::vector<element>,
            std::greater<>> elements;

    inline bool empty() const
    {
        return elements.empty();
    }

    inline void put( T item, priority_t priority )
    {
        elements.emplace( priority, item );
    }

    // returns and removes the item with the highest priority
    T get()
    {
        T itemHighestPriority = elements.top().second;
        elements.pop();
        return itemHighestPriority;
    }
};


// defining a hash function in order to use PathNode in an unordered map
template<>
struct std::hash<PathNode>
{
    std::size_t operator()( const PathNode &node ) const
    {
        return std::hash<int>()( node.x ) ^ (std::hash<int>()( node.y ) << 1);
    }
};


/**
 * @brief Finds the shortest path in a labyrinth using the A* algorithm.
 *
 * This function implements the A* search algorithm to find the shortest path
 * from the start position to the target position in a given labyrinth. It uses
 * a priority queue to explore nodes and reconstructs the path once the target
 * node is reached.
 * This implementation is inspired by
 *
 * @tparam S The size of the labyrinth (assumed to be a square grid of size SxS).
 * @param labyrinth A reference to the labyrinth grid where true indicates walkable areas.
 * @param startPosition A tuple representing the starting position (x, y).
 * @param targetPosition A tuple representing the target position (x, y).
 * @return Path The reconstructed path from the start to the target. If the start and
 *              target positions are the same, returns an empty path.
 * @throws std::runtime_error If there is no possible path from the start to the target.
 **/
template<size_t S>
Path findPath( const Labyrinth<S> &labyrinth, const std::tuple<size_t, size_t> &startPosition,
               const std::tuple<size_t, size_t> &targetPosition )
{
    // trivial case
    if ( targetPosition == startPosition ) return {};

    // defining directions for the labyrinth neighbors
    static std::pair<size_t, size_t> directions[4] = {std::make_pair( -1, 0 ), std::make_pair( 1, 0 ),
                                                      std::make_pair( 0, -1 ), std::make_pair( 0, 1 )};

    // initializing start and target node
    PathNode startNode{std::get<0>( startPosition ), std::get<1>( startPosition )};
    PathNode targetNode{std::get<0>( targetPosition ), std::get<1>( targetPosition )};

    // keeping the list of nodes which are tracked; the first element is the cheapest
    PairPriorityQueue<PathNode, double> nodesToBeExplored;
    nodesToBeExplored.put( startNode, 0 );

    // parent node to reconstruct the path later on
    std::unordered_map<PathNode, PathNode> parentNodes;
    // storing the costs to reach a certain node
    // (costs = heuristic (see above) + cost of parent node + 1)
    std::unordered_map<PathNode, double> nodesCost;

    // start nodes has no costs initially
    parentNodes[startNode] = startNode;
    nodesCost[startNode] = 0;

    while ( !nodesToBeExplored.empty())
    {
        // fetch the node with the lowest cost from the priority queue
        PathNode current = nodesToBeExplored.get();

        // target node was found
        if ( current == targetNode )
        {
            break;
        }

        // iterate over all neighbors
        for ( const auto &direction: directions )
        {
            PathNode next{current.x + direction.first, current.y + direction.second};

            // neighbors being outside the maze or being solid blocks cannot be walked through
            if ( !isInsideMaze( next.x, next.y, S ) || !labyrinth.get( next.x, next.y ))
            {
                continue;
            }

            // each step costs 1
            double new_cost = nodesCost[current] + 1;
            // if not inside nodesCost or the new path is cheaper -> update the costs
            if ( nodesCost.find( next ) == nodesCost.end() || new_cost < nodesCost[next] )
            {
                nodesCost[next] = new_cost;
                double priority = new_cost + heuristic( next, targetNode );
                nodesToBeExplored.put( next, priority );
                parentNodes[next] = current;
            }
        }
    }

    if ( !parentNodes.count( targetNode ))
    {
        throw std::runtime_error( "There is no possible path for this maze." );
    }

    // reconstructing the path to target
    Path path;
    for ( PathNode node = targetNode; !(node == startNode); node = parentNodes[node] )
    {
        path.pushNode( node.x, node.y );
    }
    // adding the start
    path.pushNode( startNode.x, startNode.y );
    // reverse the path to start from the beginning
    path.reverse();
    return path;
}

