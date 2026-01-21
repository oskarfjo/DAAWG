#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

#include "Astar.h"
#include "types.h"
#include "GeoLoader.h"

using namespace CalculatedPath;

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
    Waypoint wstart = {62.206767, 6.472522, 0.0f}; // Sæbø fotballbane
    Waypoint wgoal = {62.230278, 6.435130, 0.0f}; // toppen av dalegubben

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
        std::cout << "No path found." << std::endl;
    } else {

        ////////////////////////////////////////////////
        ////// Prints path to terminal for dev /////////
        ////////////////////////////////////////////////

        std::cout << "Path found with " << path.size() << " nodes." << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            Waypoint wp;

            loader.node_to_waypoint(currentMap, path[i], wp);

            std::cout << "WP " << i << ": "
                    << std::fixed << std::setprecision(6) 
                    << wp.lat << ", " << wp.lon 
                    << " | Alt: " << wp.alt << "m" << std::endl;
        }
    }
    
    return 0;
}