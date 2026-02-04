#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "searchPattern.h"
#include "types.h"
#include "GeoLoader.h"

using namespace CalculatedPath;

void exportMissionFileTxt(const std::vector<Waypoint>& path);
void exportMissionFilePlan(const std::vector<Waypoint>& path);

int main() {
    std::cout << "Drone Search Pattern Program initialized!" << std::endl;
    
    GeoLoader loader;
    MapData currentMap;

    std::string mapPath = "../maps/tifs/lowerRight.tif"; // Path to the .tif file
    
    std::cout << "Loading map from: " << mapPath << std::endl;
    
    if (!loader.load_map(mapPath, currentMap)) {
        std::cerr << "Failed to load map data! Check file path." << std::endl;
        return 1;
    }
    std::cout << "Map loaded successfully (" << currentMap.width << "m x " << currentMap.height << "m)" << std::endl;

    //// DEFINE AVALANCHE LOCATION ////
    Waypoint wAvalanche;
    wAvalanche.lat = 62.3072221;
    wAvalanche.lon = 6.836761;

    int avalancheLength = 200; // m
    int avalancheWidth = 85; // m

    Node iAvalanche;

    std::cout << "Mapping waypoints to nodes..." << std::endl;
    if (!loader.waypoint_to_node(currentMap, wAvalanche, iAvalanche)) {
        std::cerr << "Avalanche is outside the loaded map area!" << std::endl;
        return 1;
    }

    // Generates the search pattern
    std::vector<Node> missionNodes = searchPattern(iAvalanche, avalancheLength, avalancheWidth, currentMap);

    if (missionNodes.empty()) {
        std::cout << "Pattern generation error! <EMPTY VECTOR>" << std::endl;
        return 1;
    }

    // converts Node vector to Waypoint vector
    std::vector<Waypoint> missionWaypoints;
    missionWaypoints.reserve(missionNodes.size());

    for (size_t i = 0; i < missionNodes.size(); ++i) {

        Node currentNode = missionNodes[i];
        
        Waypoint currentWaypoint;

        if (loader.node_to_waypoint(currentMap, currentNode, currentWaypoint)) {
            missionWaypoints.push_back(currentWaypoint);
        }
    }
    
    exportMissionFileTxt(missionWaypoints);

    return 0;
}

void exportMissionFileTxt(const std::vector<Waypoint>& path) {
    std::ofstream outfile("../maps/mission.txt");

    if (!outfile.is_open()) {
        std::cerr << "Error: Could not create search pattern file." << std::endl;
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
            wp.coordinate_frame = 0; 
        }

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
    std::cout << "Search pattern file generated successfully." << std::endl;
}


void exportMissionFilePlan(const std::vector<Waypoint>& path) {
    std::ofstream outfile("../maps/searchPattern.plan");

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