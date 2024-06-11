#pragma once

#include <iostream>
#include <array>

const static Labyrinth labyrinth ({
    false, false, true, false,
    false, true, true, false,
    false, false, true, false,
    false, false, true, false,
});  

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

