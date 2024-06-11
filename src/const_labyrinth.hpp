#pragma once
#include <array>
#include "labyrinth.hpp"

const static Labyrinth LABYRINTH ({
                                          false, false, true, false,
                                          false, true, true, false,
                                          false, false, true, false,
                                          false, false, true, false,
                                  });

constexpr size_t LABYRINTH_SIZE = 4;