#pragma once

#include <array>

/**
 * @brief datatype for the labyrinth
 * holds an array of bools (true: air, false: block)
 * the array holds 2d map, column first
*/
template<size_t S>
class Labyrinth {
public: 
    Labyrinth(const std::array<bool, S * S> &labyrinth)
        : m_cells(labyrinth)
{
};

    bool get(size_t x, size_t y) const
    {
        return m_cells[y * S + x];
    } 

    size_t getSize() const 
    {
        return S;
    };

private: 
    std::array<bool, S * S> m_cells = {};
}; 

