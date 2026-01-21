#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cmath>

namespace CalculatedPath {

    struct Waypoint // formated for pixhawk
    {
        double lat;
        double lon;
        float alt;
    };

    struct Node // formatted for A*
    {
        int x, y; // Grid index
        float alt;
    };
        
    struct Coordinate
    {
        int x, y;
        bool operator<(const Coordinate& other) const {
            if (x != other.x) return x < other.x;
            return y < other.y;
        }
    };
}

#endif