#ifndef SEARCH_H
#define SEARCH_H

#include <vector>
#include "types.h"
#include "GeoLoader.h"

namespace CalculatedPath {
    // Now returns Waypoints directly, takes a center Waypoint instead of Node
    std::vector<Waypoint> searchPattern(Waypoint center, int length, int width, const MapData& map, const GeoLoader& loader);
}

#endif