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

void exportMissionFileTxt(const std::vector<Waypoint>& path);

void exportMissionFilePlan(const std::vector<Waypoint>& path);

int main() {
    std::cout << "Drone Pathfinder Program initialized!" << std::endl;
    
    GeoLoader loader;
    MapData currentMap;

    std::string mapPath = "../maps/tifs/upperRight.tif"; // Path to the .tif file
    
    std::cout << "Loading map from: " << mapPath << std::endl;
    
    if (!loader.load_map(mapPath, currentMap)) {
        std::cerr << "Failed to load map data! Check file path." << std::endl;
        return 1;
    }
    std::cout << "Map loaded successfully (" << currentMap.width << "m x " << currentMap.height << "m)" << std::endl;

    //// DEFINE START AND END ////
    Waypoint wstart;
    wstart.lat = 62.441025;
    wstart.lon = 6.420500;

    Waypoint wgoal;
    wgoal.lat = 62.439891;
    wgoal.lon = 6.419728;

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
    } else {
        std::cout << "Path found (" << gridPath.size() << " nodes)." << std::endl;
        
        // SIMPLIFY
        std::vector<Node> simplePath = BSpline::Simplify(gridPath, 2.0); 
        
        // SMOOTH
        std::vector<Waypoint> smoothPath = BSpline::Generate(simplePath, loader, currentMap, 3);
        
        std::cout << "Creating mission file" << std::endl;
        exportMissionFilePlan(smoothPath); 
    }
}

// Rewritten Export Function to accept pre-calculated Waypoints
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
        wp.alt += 15;
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
    std::ofstream outfile("../maps/mission.plan");

    if (!outfile.is_open()) {
        std::cerr << "Error: Could not create mission plan file." << std::endl;
        return;
    }

    std::ostringstream itemsJson;
    itemsJson << std::fixed << std::setprecision(8);

    for (size_t i = 0; i < path.size(); ++i) {
        Waypoint wp = path[i];

        int command = 16;  // MAV_CMD_NAV_WAYPOINT
        int frame = 0;  // 0 = MAV_FRAME_GLOBAL
        bool autoContinue = true;
        double altitude = wp.alt + 15;

        if (i > 0) itemsJson << ",";
        
        itemsJson << R"(
        {
            "AMSLAltAboveTerrain": null,
            "Altitude": )" << altitude << R"(,
            "AltitudeMode": 2,
            "autoContinue": )" << (autoContinue ? "true" : "false") << R"(,
            "command": )" << command << R"(,
            "doJumpId": )" << (i + 1) << R"(,
            "frame": )" << frame << R"(,
            "params": [
                )" << wp.pause << R"(,
                )" << wp.accept_radius << R"(,
                0,
                null,
                )" << wp.lat << R"(,
                )" << wp.lon << R"(,
                )" << altitude << R"(
            ],
            "type": "SimpleItem"
        })";
    }

    // Get home position from first waypoint (or use defaults)
    double homeLat = path.empty() ? 0.0 : path[0].lat;
    double homeLon = path.empty() ? 0.0 : path[0].lon;
    double homeAlt = path.empty() ? 0.0 : path[0].alt;

    outfile << std::fixed << std::setprecision(8);
    outfile << R"({
    "fileType": "Plan",
    "geoFence": {
        "circles": [],
        "polygons": [],
        "version": 2
    },
    "groundStation": "QGroundControl",
    "mission": {
        "cruiseSpeed": 15,
        "firmwareType": 12,
        "globalPlanAltitudeMode": 2,
        "hoverSpeed": 5,
        "items": [)" << itemsJson.str() << R"(
        ],
        "plannedHomePosition": [
            )" << homeLat << R"(,
            )" << homeLon << R"(,
            )" << homeAlt << R"(
        ],
        "vehicleType": 20,
        "version": 2
    },
    "rallyPoints": {
        "points": [],
        "version": 2
    },
    "version": 1
})";

    outfile.close();
    std::cout << "Mission plan file generated successfully." << std::endl;
}