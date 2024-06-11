#pragma once
#include <array>
#include "labyrinth.hpp"

const static Labyrinth<12> LABYRINTH ({

	false, false, false, false, false, false, false, false, false, false, false, false, 
	false, true, true, true, true, true, true, true, true, true, true, false, 
	false, true, false, false, false, false, true, true, false, false, true, false, 
	false, true, true, true, true, false, true, true, true, false, true, false, 
	false, true, true, false, true, false, true, false, false, false, true, false, 
	false, true, false, false, true, false, true, true, true, true, true, false, 
	false, false, false, false, true, true, true, true, true, false, true, false, 
	false, true, true, false, true, false, false, false, true, false, true, false, 
	false, true, true, true, true, true, true, true, false, false, false, false, 
	false, false, false, true, true, true, true, true, false, true, true, false, 
	false, true, true, true, true, false, true, true, false, false, true, false, 
	false, false, false, false, false, false, false, false, false, false, false, false
});
