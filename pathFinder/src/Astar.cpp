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
        float dz = static_cast<float>(a.z - b.z);
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}

std::vector<Node> CalculatedPath::A_star(Node start, Node goal, const MapData& map) {
    
    std::list<PathNode> allNodes; 
    std::priority_queue<PathNode*, std::vector<PathNode*>, CompareNode> openSet;
    std::map<Coordinate, float> visited;

    // INIT - convert input Node to internal PathNode
    PathNode startInternal;
    int roundedStartAlt = std::round(start.alt);
    startInternal.x = start.x;
    startInternal.y = start.y;
    startInternal.z = roundedStartAlt;
    startInternal.gCost = 0.0f;
    
    // Store goal as PathNode for distance calculations
    PathNode goalInternal;
    int roundedGoalAlt = std::round(goal.alt);
    goalInternal.x = goal.x;
    goalInternal.y = goal.y;
    goalInternal.z = roundedGoalAlt;

    allNodes.push_back(startInternal);
    PathNode* startPtr = &allNodes.back();
    startPtr->hCost = calculateDistance3D(*startPtr, goalInternal);
    
    openSet.push(startPtr);
    visited[{start.x, start.y, roundedStartAlt}] = 0.0f;

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
                    int nextZ = current->z + (zOffset * 3);

                    int groundAlt = std::ceil(map.get_elevation(nextX, nextY)); // To see if it attempts to go bellow the map

                    if (!map.is_valid(nextX, nextY) || nextZ <= groundAlt || nextZ >= groundAlt+300) continue;

                    PathNode neighbor;
                    neighbor.x = nextX;
                    neighbor.y = nextY;
                    neighbor.z = nextZ;

                    float altCostMultiplier;
                    if (nextZ <= groundAlt+75) {
                        altCostMultiplier = 2.0f;
                    } else if (nextZ <= groundAlt+125) {
                        altCostMultiplier = 1.0f;
                    } else {
                        altCostMultiplier = 1.2f;
                    }

                    float moveCost = calculateDistance3D(*current, neighbor) * altCostMultiplier;
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

                        if (false) {
                            std::cout << "allNodes length: " << allNodes.size() << 
                            "\t openSet lenght: " << openSet.size() <<
                            "\t hCost current: " << neighborPtr->hCost <<
                            "\t fCost current: " << neighborPtr->fCost() <<
                            "\t altCostMultiplier " << altCostMultiplier <<
                            std::endl;
                        }
                    }
                }
            }
        }
    }

    return {}; 
}