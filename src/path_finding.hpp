#pragma once

#include <iostream>
#include <tuple>
#include "labyrinth.hpp"
#include "path.hpp"

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
Path findPath(const Labyrinth &labyrinth, const std::tuple<size_t, size_t> &startPosition,
              const std::tuple<size_t, size_t> &targetPosition);
