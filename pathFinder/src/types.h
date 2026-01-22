#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cmath>

namespace CalculatedPath {

    struct Waypoint // formated for pixhawk
    {
        int wp_nr;
        int wp_current;
        int coordinate_frame;
        int action;
        int pause;
        int accept_radius;
        int passthrough_radius;
        int yaw_wing;
        double lat;
        double lon;
        float alt;
        int autocontinue = 1;

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