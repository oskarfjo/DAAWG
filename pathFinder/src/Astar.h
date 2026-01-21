#ifndef ASTAR_H
#define ASTAR_H

#include <vector>

#include "types.h"
#include "GeoLoader.h"

namespace CalculatedPath {
    std::vector<Node> A_star(Node start, Node goal, const MapData& map);
}

#endif