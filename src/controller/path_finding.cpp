#include <algorithm>
#include <vector>
#include "path_finding.hpp"


bool contains( const std::vector<PathNode> &visitedNodes, const PathNode &element )
{
    return std::find( visitedNodes.begin(), visitedNodes.end(), element ) != visitedNodes.end();
}


bool isInsideMaze( int x, int y, size_t labyrinth_size )
{
    return (x >= 0 && x < labyrinth_size && y >= 0 && y < labyrinth_size);
}


bool isDirectNeighbor( size_t dx, size_t dy )
{
    return ((dx == 0 || dy == 0) && dx != dy);
}
