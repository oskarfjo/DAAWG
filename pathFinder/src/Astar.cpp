#include "Astar.h"
#include <cmath>
#include <queue>
#include <vector>
#include <list>
#include <algorithm>
#include <map>
#include <iostream>

using namespace CalculatedPath;

namespace { // Makes local namespace for simplicity
    struct PathNode {
        int x, y, z;
        float gCost = 0.0f;
        float hCost = 0.0f;
        PathNode* parent = nullptr;
        
        float fCost() const { return gCost + hCost; }
        
        bool samePosition(int ox, int oy) const {
            return x == ox && y == oy;
        }
    };

    struct CompareNode {
        bool operator()(PathNode* a, PathNode* b) {
            return a->fCost() > b->fCost();
        }
    };

    float calculateDistance3D(const PathNode& a, const PathNode& b) {
        float dx = static_cast<float>(a.x - b.x) * 10.0f;
        float dy = static_cast<float>(a.y - b.y) * 10.0f;
        float dz = static_cast<float>(a.z - b.z) * 2.0f;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}

std::vector<Node> CalculatedPath::A_star(Node start, Node goal, const MapData& map) {
    
    std::list<PathNode> allNodes; 
    std::priority_queue<PathNode*, std::vector<PathNode*>, CompareNode> openSet;
    std::map<Coordinate, float> visited;
    
    int roundedStartAlt = std::round(start.alt);
    int roundedGoalAlt = std::round(goal.alt);

    // Store goal as PathNode for distance calculations
    PathNode goalInternal;
    goalInternal.x = goal.x;
    goalInternal.y = goal.y;
    goalInternal.z = roundedGoalAlt + 25;

    // Generates different start node options with different quad ascent altitudes
    for (int initZoffset = 20; initZoffset < 100; initZoffset += 5) {

        PathNode startOption;
        startOption.x = start.x;
        startOption.y = start.y;
        startOption.z = roundedStartAlt + initZoffset;
        startOption.gCost = initZoffset * 10;
        startOption.hCost = calculateDistance3D(startOption, goalInternal);

        allNodes.push_back(startOption);
        PathNode* startPtr = &allNodes.back();
        
        openSet.push(startPtr);
        visited[{startOption.x, startOption.y, startOption.z}] = startOption.gCost;
    }


    // SEARCH LOOP
    while (!openSet.empty()) {
        
        PathNode* current = openSet.top();
        openSet.pop();

        // Goal reached?
        if (current->samePosition(goal.x, goal.y)) {
            // Convert PathNode chain to Node vector
            std::vector<Node> path;
            PathNode* tracer = current;
            
            while (tracer != nullptr) {
                Node n;
                n.x = tracer->x;
                n.y = tracer->y;
                n.alt = tracer->z;
                path.push_back(n);
                tracer = tracer->parent;
            }
            
            std::reverse(path.begin(), path.end());
            return path;
        }

        // GENERATE NEIGHBORS
        for (int zOffset = -1; zOffset <= 1; zOffset++) {
            for (int xOffset = -1; xOffset <= 1; xOffset++) {
                for (int yOffset = -1; yOffset <= 1; yOffset++) {
                    
                    if (xOffset == 0 && yOffset == 0 && zOffset == 0) continue;

                    int nextX = current->x + xOffset;
                    int nextY = current->y + yOffset;
                    int nextZ = current->z + (zOffset * 2);

                    int groundAlt = std::ceil(map.get_elevation(nextX, nextY));

                    if (!map.is_valid(nextX, nextY) || nextZ <= groundAlt || nextZ >= groundAlt+200) continue;

                    PathNode neighbor;
                    neighbor.x = nextX;
                    neighbor.y = nextY;
                    neighbor.z = nextZ;

                    int safetyRadius = 4; // 40m (4 nodes)
                    float closestDistance = nextZ - groundAlt;
                    
                    for (int xSphereOffset=-safetyRadius; xSphereOffset <=safetyRadius; xSphereOffset += 2) {
                        for (int ySphereOffset=-safetyRadius; ySphereOffset <=safetyRadius; ySphereOffset += 2) {
                            
                            if (xSphereOffset == 0 && ySphereOffset == 0) continue;

                            int checkX = nextX + xSphereOffset;
                            int checkY = nextY + ySphereOffset;

                            if (!map.is_valid(checkX, checkY)) continue;

                            int checkAlt = std::ceil(map.get_elevation(checkX, checkY));

                            float diffX = static_cast<float>(checkX - nextX) * 10.0f;
                            float diffY = static_cast<float>(checkY - nextY) * 10.0f;
                            float diffZ;

                            if (checkAlt >= nextZ) {
                                diffZ = 0;
                            } else {
                                diffZ = (nextZ - checkAlt) * 2.0f;
                            }

                            float calculatedDistance = std::sqrt(
                                std::pow(diffX, 2) +
                                std::pow(diffY, 2) +
                                std::pow(diffZ, 2)
                            );

                            if (calculatedDistance < closestDistance) closestDistance = calculatedDistance;
                        }
                    }

                    float altCostMultiplier;
                    if (nextZ <= groundAlt+20) {
                        altCostMultiplier = 20.0f;
                    } else if (nextZ <= groundAlt+35) {
                        altCostMultiplier = 10.0f;
                    } else if (nextZ <= groundAlt+70) {
                        altCostMultiplier = 3.0f;
                    } else if (nextZ <= groundAlt+100) {
                        altCostMultiplier = 1.0f;
                    } else {
                        altCostMultiplier = 5.0f;
                    }

                    float proxCostMultiplier;
                    if (closestDistance <= 20) {
                        proxCostMultiplier = 10.0f;
                    } else if (closestDistance <= 40) {
                        proxCostMultiplier = 5.0f;
                    } else {
                        proxCostMultiplier = 1.0f;
                    }

                    float moveCost = calculateDistance3D(*current, neighbor) * altCostMultiplier * proxCostMultiplier;
                    float newGCost = current->gCost + moveCost;

                    Coordinate coord = {nextX, nextY, nextZ};
                    if (visited.find(coord) == visited.end() || newGCost < visited[coord]) {
                        
                        allNodes.push_back(neighbor);
                        PathNode* neighborPtr = &allNodes.back();

                        visited[coord] = newGCost;
                        neighborPtr->gCost = newGCost;
                        neighborPtr->hCost = calculateDistance3D(*neighborPtr, goalInternal);
                        neighborPtr->parent = current;

                        openSet.push(neighborPtr);

                        if (false) { // debug
                            std::cout << "allNodes length: " << allNodes.size() << 
                            "\t openSet lenght: " << openSet.size() <<
                            "\t hCost current: " << neighborPtr->hCost <<
                            "\t fCost current: " << neighborPtr->fCost() <<
                            "\t altCostMultiplier " << altCostMultiplier <<
                            "\t proxCostMultiplier " << proxCostMultiplier <<
                            std::endl;
                        }
                    }
                }
            }
        }
    }

    return {}; 
}