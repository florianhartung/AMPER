#include "path_finding.hpp"


bool isInsideMaze( int x, int y, size_t labyrinth_size )
{
    return (x >= 0 && x < labyrinth_size && y >= 0 && y < labyrinth_size);
}

// Manhattan distance as a heuristic functions -> for estimating the path in A*
int heuristic( const PathNode &a, const PathNode &b )
{
    return abs( static_cast<int>(a.x) - static_cast<int>(b.x))
           + abs( static_cast<int>(a.y) - static_cast<int>(b.y));
}