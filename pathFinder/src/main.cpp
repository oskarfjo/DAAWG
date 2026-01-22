#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>

#include "Astar.h"
#include "types.h"
#include "GeoLoader.h"

using namespace CalculatedPath;

void exportMissionFile(const std::vector<Node>& path, GeoLoader& loader, const MapData& currentMap);

int main() {
    std::cout << "Drone Pathfinder Program initialized!" << std::endl;
    
    GeoLoader loader;
    MapData currentMap;

    std::string mapPath = "../maps/orsta_hoyde_10m/dom10/data/dom10_6900_2_10m_z33.tif"; // Path to the .tif file
    
    std::cout << "Loading map from: " << mapPath << std::endl;
    
    if (!loader.load_map(mapPath, currentMap)) {
        std::cerr << "Failed to load map data! Check file path." << std::endl;
        return 1;
    }
    std::cout << "Map loaded successfully (" << currentMap.width << "x" << currentMap.height << ")" << std::endl;

    //// DEFINE START AND END ////
    Waypoint wstart; // Sæbø fotballbane
    wstart.lat = 62.206767;
    wstart.lon = 6.472522;
    wstart.alt = 4.6;

    Waypoint wgoal; // toppen av dalegubben
    wgoal.lat = 62.230278;
    wgoal.lon = 6.435130;
    wgoal.alt = 0.0;

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
    
    std::vector<Node> path = A_star(istart, igoal, currentMap); // Runs the A* pathfinding algorithm

    if (path.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        std::cout << "Path found\n" << "Creating mission file" << std::endl;
        exportMissionFile(path, loader, currentMap);
    }
    
    return 0;
}

void exportMissionFile(const std::vector<Node>& path, GeoLoader& loader, const MapData& currentMap) { // Assuming path contains Nodes
    std::ofstream outfile("../maps/mission.txt");

    if (!outfile.is_open()) {
        std::cerr << "Error: Could not create mission file." << std::endl;
        return;
    }

    // Header
    outfile << "QGC WPL 110" << "\n";

    // Iterate through the .txt and write entries
    for (size_t i = 0; i < path.size(); ++i) {
        Waypoint wp;

        wp.wp_nr = i;

        // TODO: make this less goofy
        if (wp.wp_nr == 0) {
            wp.wp_current = 1;
            wp.coordinate_frame = 0; // Global frame for home
        } else {
            wp.wp_current = 0;
            wp.coordinate_frame = 3; // Relative altitude frame
        }

        // TODO: Fix entire Waypoint data setting
        wp.action = 16;
        wp.pause = 0;
        wp.accept_radius = 5;
        wp.passthrough_radius = 0;
        wp.yaw_wing = 0;
        wp.autocontinue = 1;

        loader.node_to_waypoint(currentMap, path[i], wp);

        // Write to file
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