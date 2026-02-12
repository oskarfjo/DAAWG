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
        int autocontinue;
    };

    struct Node
    {
        int x, y; // Grid index
        float alt;
    };

    struct Coordinate
    {
        double lat, lon;
    };
}

#endif