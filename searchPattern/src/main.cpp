#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>

#include "searchPattern.h"
#include "types.h"
#include "GeoLoader.h"

using namespace CalculatedPath;

void exportMissionFile(const std::vector<Waypoint>& path);

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
    wAvalanche.lat = 62.3082386;
    wAvalanche.lon = 6.8446325;
    wAvalanche.alt = 1138.4;

    int avalancheLength = 183; // m
    int avalancheWidth = 71; // m

    Node iAvalanche;

    std::cout << "Mapping waypoints to nodes..." << std::endl;
    if (!loader.waypoint_to_node(currentMap, wAvalanche, iAvalanche)) {
        std::cerr << "Avalanche is outside the loaded map area!" << std::endl;
        return 1;
    }

    std::vector<Node> missionNodes = searchPattern(iAvalanche, avalancheLength, avalancheWidth, currentMap);

    if (missionNodes.empty()) {
        std::cout << "Pattern generation error! <EMPTY VECTOR>" << std::endl;
        return 1;
    }

    std::vector<Waypoint> missionWaypoints;
    //missionWaypoints.resize(missionNodes.size());

    for (size_t i = 0; i < missionNodes.size(); ++i) {
        Node currentNode = missionNodes[i];
        Waypoint currentWaypoint;

        if (loader.node_to_waypoint(currentMap, currentNode, currentWaypoint)) {
            missionWaypoints.push_back(currentWaypoint);
        }
    }
    
    exportMissionFile(missionWaypoints);

    return 0;
}

void exportMissionFile(const std::vector<Waypoint>& path) {
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
            wp.coordinate_frame = 0; 
        } else {
            wp.wp_current = 0;
            wp.coordinate_frame = 3; 
        }
        
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