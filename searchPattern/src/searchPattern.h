#ifndef SEARCH_H
#define SEARCH_H

#include <vector>

#include "types.h"
#include "GeoLoader.h"

namespace CalculatedPath {
    std::vector<Node> searchPattern(Node start, int length, int width, const MapData& map);
}

#endif