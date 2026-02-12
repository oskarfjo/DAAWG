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

    std::string mapPath = "../maps/tifs/lowerRight.tif";
    
    if (!loader.load_map(mapPath, currentMap)) {
        return 1;
    }

    //// DEFINE AVALANCHE LOCATION ////
    Waypoint wAvalanche;
    wAvalanche.lat = 62.3072221; //62.312669; 62.3127336;
    wAvalanche.lon = 6.836761; // 6.8426098; 6.8367186;
    wAvalanche.alt = loader.get_elevation_at_coordinate(currentMap, wAvalanche.lat, wAvalanche.lon); // Get center alt

    int avalancheLength = 200; // m
    int avalancheWidth = 85; // m

    std::vector<Waypoint> missionWaypoints = searchPattern(wAvalanche, avalancheLength, avalancheWidth, currentMap, loader);

    if (missionWaypoints.empty()) {
        std::cout << "Pattern generation error! <EMPTY VECTOR>" << std::endl;
        return 1;
    }
    
    //exportMissionFileTxt(missionWaypoints);
    exportMissionFilePlan(missionWaypoints);

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
        }

        wp.coordinate_frame = 0; 
        wp.action = 16;
        wp.pause = 0;
        wp.accept_radius = 5;
        wp.passthrough_radius = 5;
        wp.yaw_wing = 0;
        wp.autocontinue = 1;

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

        int command = 16;
        int frame = 0;
        wp.accept_radius = 5;
        wp.pause = 0;
        bool autoContinue = true;

        if (i > 0) itemsJson << ",";
        
        itemsJson << R"(
        {
            "AMSLAltAboveTerrain": null,
            "Altitude": )" << wp.alt << R"(,
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
                )" << wp.alt << R"(
            ],
            "type": "SimpleItem"
        })";
    }

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
            )" << path[0].lat << R"(,
            )" << path[0].lon << R"(,
            )" << path[0].alt << R"(
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