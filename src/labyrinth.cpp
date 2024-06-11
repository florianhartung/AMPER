#include "labyrinth.hpp"

Labyrinth::Labyrinth(const std::array<bool, LABYRINTH_SIZE * LABYRINTH_SIZE> &labyrinth)
    : m_cells(labyrinth)
{
}

bool Labyrinth::get(size_t x, size_t y) const
{
    return m_cells[y * LABYRINTH_SIZE + x];
}
