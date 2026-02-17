#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "Astar.h"
#include "types.h"
#include "BSplein.h"
#include "GeoLoader.h"

using namespace CalculatedPath;

double toRad(double degree) {
    return degree/180 * M_PI;
}

double calculateDistance(double lat1, double long1, double lat2, double long2) {
    double dist;
    dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long1 - long2));
    dist = acos(dist);
    dist = 6371000 * dist;
    return dist;
}

void exportMissionFileTxt(const std::vector<Waypoint>& path);

void exportMissionFilePlan(const std::vector<Waypoint>& path);

int main() {
    std::cout << "Drone Pathfinder Program initialized!" << std::endl;
    
    GeoLoader loader;
    MapData currentMap;

    std::string mapPath = "../maps/tifs/lowerRight.tif"; // Path to the .tif file
    
    std::cout << "Loading map from: " << mapPath << std::endl;
    
    if (!loader.load_map(mapPath, currentMap)) {
        std::cerr << "Failed to load map data! Check file path." << std::endl;
        return 1;
    }
    std::cout << "Map loaded successfully (" << currentMap.width << "m x " << currentMap.height << "m)" << std::endl;

    //// DEFINE START AND END ////
    Waypoint wstart;
    wstart.lat = 62.290748;
    wstart.lon = 6.844344;

    Waypoint wgoal;
    wgoal.lat = 62.2989958;
    wgoal.lon = 6.8655505;

    Node istart, igoal;

    std::cout << "Mapping waypoints to nodes..." << std::endl;
    if (!loader.waypoint_to_node(currentMap, wstart, istart) || 
        !loader.waypoint_to_node(currentMap, wgoal, igoal)) {
        std::cerr << "Start or Goal point is outside the loaded map area!" << std::endl;
        return 1;
    }
    
    std::cout << "Start Node: [" << istart.x << ", " << istart.y << "] Alt: " << istart.alt << std::endl;
    std::cout << "Goal Node: [" << igoal.x << ", " << igoal.y << "] Alt: " << igoal.alt << std::endl;

    std::cout << "Calculating path..." << std::endl;
    
    std::vector<Node> gridPath = A_star(istart, igoal, currentMap);

    if (gridPath.empty()) {
        std::cout << "No path found" << std::endl;
        return 1;
    } else {
        std::cout << "Path found (" << gridPath.size() << " nodes)." << std::endl;
        
        // SIMPLIFY
        std::vector<Node> simplePath = BSpline::Simplify(gridPath, 10.0); 
        
        // SMOOTH
        std::vector<Waypoint> smoothPath = BSpline::Generate(simplePath, loader, currentMap, 3);

        std::cout << "Simplified to (" << smoothPath.size() << " waypoints)." << std::endl;

        if (false) { // debug
            for (int i = 0; i < smoothPath.size(); i++) {
                double distance = calculateDistance(smoothPath[i].lat, smoothPath[i].lon, smoothPath[i+1].lat, smoothPath[i+1].lon);
                double altDiff = smoothPath[i+1].alt - smoothPath[i].alt;
                double angle = std::asin(altDiff/distance) * (180/M_PI);
                std::cout << "WP" << i << ": Angle = " << angle << "°" << std::endl;
            }
        }
        
        std::cout << "Creating mission file" << std::endl;
        exportMissionFilePlan(smoothPath);
        exportMissionFileTxt(smoothPath);
        return 0;
    }
}

void exportMissionFileTxt(const std::vector<Waypoint>& path) {
    std::ofstream outfile("../maps/mission.txt");

    if (!outfile.is_open()) {
        std::cerr << "Error: Could not create mission file." << std::endl;
        return;
    }

    // Pixhawk mission file header
    outfile << "QGC WPL 110" << "\n";

    for (size_t i = 0; i < path.size(); ++i) {
        // Copy the waypoint to modify the sequence numbers
        Waypoint wp = path[i]; 

        wp.wp_nr = i;

        if (wp.wp_nr == 0) {
            wp.wp_current = 1; 
        } else {
            wp.wp_current = 0;
        }
        
        wp.coordinate_frame = 0;
        wp.action = 16;
        wp.pause = 0;
        wp.passthrough_radius = 0;
        wp.yaw_wing = 0;

        outfile << std::fixed << std::setprecision(8) 
                << wp.wp_nr << "\t"
                << wp.wp_current << "\t"
                << wp.coordinate_frame << "\t"
                << wp.action << "\t"
                << wp.pause << "\t"
                << wp.accept_radius << "\t"
                << wp.passthrough_radius << "\t"
                << wp.yaw_wing << "\t"
                << wp.lat << "\t"
                << wp.lon << "\t"
                << wp.alt << "\t"
                << wp.autocontinue
                << "\n";
    }

    outfile.close();
    std::cout << "Mission file generated successfully." << std::endl;
}

void exportMissionFilePlan(const std::vector<Waypoint>& path) {
    std::ofstream outfile("../maps/path.plan");

    if (!outfile.is_open() || path.empty()) {
        std::cerr << "Error: Could not create mission plan file." << std::endl;
        return;
    }

    std::ostringstream itemsJson;
    itemsJson << std::fixed << std::setprecision(8);

    int jumpId = 1;

    // VTOL TAKEOFF
    itemsJson << R"(
        {
            "AMSLAltAboveTerrain": null,
            "Altitude": )" << path[0].alt << R"(,
            "AltitudeMode": 2,
            "autoContinue": true,
            "command": 84,
            "doJumpId": )" << jumpId++ << R"(,
            "frame": 0,
            "params": [0, 0, 0, null, )" << path[0].lat << ", " << path[0].lon << ", " << path[0].alt << R"(],
            "type": "SimpleItem"
        })";

    // TRANSIT WAYPOINTS
    for (size_t i = 0; i < path.size(); ++i) {
        itemsJson << R"(,
        {
            "AMSLAltAboveTerrain": null,
            "Altitude": )" << path[i].alt << R"(,
            "AltitudeMode": 2,
            "autoContinue": true,
            "command": 16,
            "doJumpId": )" << jumpId++ << R"(,
            "frame": 0,
            "params": [
                )" << path[i].pause << R"(,
                )" << path[i].accept_radius << R"(,
                0,
                null,
                )" << path[i].lat << R"(,
                )" << path[i].lon << R"(,
                )" << path[i].alt << R"(
            ],
            "type": "SimpleItem"
        })";
    }

    outfile << std::fixed << std::setprecision(8);
    outfile << R"({
    "fileType": "Plan",
    "geoFence": { "circles": [], "polygons": [], "version": 2 },
    "groundStation": "QGroundControl",
    "mission": {
        "cruiseSpeed": 15,
        "firmwareType": 12,
        "globalPlanAltitudeMode": 2,
        "hoverSpeed": 5,
        "items": [)" << itemsJson.str() << R"(
        ],
        "plannedHomePosition": [
            )" << path[0].lat << R"(,
            )" << path[0].lon << R"(,
            )" << path[0].alt << R"(
        ],
        "vehicleType": 20,
        "version": 2
    },
    "rallyPoints": { "points": [], "version": 2 },
    "version": 1
})";

    outfile.close();
    std::cout << "Mission plan file generated successfully." << std::endl;
}