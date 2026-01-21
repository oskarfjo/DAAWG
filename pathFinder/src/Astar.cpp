#include "Astar.h"
#include <cmath>
#include <queue>
#include <vector>
#include <list>
#include <algorithm>
#include <map>

using namespace CalculatedPath;

namespace { // Makes local namespace for simplicity
    struct PathNode {
        int x, y;
        float alt;
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
        float dx = static_cast<float>(a.x - b.x);
        float dy = static_cast<float>(a.y - b.y);
        float dz = a.alt - b.alt;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}

std::vector<Node> CalculatedPath::A_star(Node start, Node goal, const MapData& map) {
    
    std::list<PathNode> allNodes; 
    std::priority_queue<PathNode*, std::vector<PathNode*>, CompareNode> openSet;
    std::map<Coordinate, float> visited;

    // INIT - convert input Node to internal PathNode
    PathNode startInternal;
    startInternal.x = start.x;
    startInternal.y = start.y;
    startInternal.alt = start.alt;
    startInternal.gCost = 0.0f;
    
    // Store goal as PathNode for distance calculations
    PathNode goalInternal;
    goalInternal.x = goal.x;
    goalInternal.y = goal.y;
    goalInternal.alt = goal.alt;

    allNodes.push_back(startInternal);
    PathNode* startPtr = &allNodes.back();
    startPtr->hCost = calculateDistance3D(*startPtr, goalInternal);
    
    openSet.push(startPtr);
    visited[{start.x, start.y}] = 0.0f;

    // SEARCH LOOP
    while (!openSet.empty()) {
        
        PathNode* current = openSet.top();
        openSet.pop();

        // Goal reached?
        if (current->samePosition(goal.x, goal.y)) {
            // Convert PathNode chain back to Node vector
            std::vector<Node> path;
            PathNode* tracer = current;
            
            while (tracer != nullptr) {
                Node n;
                n.x = tracer->x;
                n.y = tracer->y;
                n.alt = tracer->alt;
                path.push_back(n);
                tracer = tracer->parent;
            }
            
            std::reverse(path.begin(), path.end());
            return path;
        }

        // GENERATE NEIGHBORS
        for (int xOffset = -1; xOffset <= 1; xOffset++) {
            for (int yOffset = -1; yOffset <= 1; yOffset++) {
                
                if (xOffset == 0 && yOffset == 0) continue;

                int nextX = current->x + xOffset;
                int nextY = current->y + yOffset;

                if (!map.is_valid(nextX, nextY)) continue;

                PathNode neighbor;
                neighbor.x = nextX;
                neighbor.y = nextY;
                neighbor.alt = map.get_elevation(nextX, nextY);

                float moveCost = calculateDistance3D(*current, neighbor);
                float newGCost = current->gCost + moveCost;

                Coordinate coord = {nextX, nextY};
                if (visited.find(coord) == visited.end() || newGCost < visited[coord]) {
                    
                    allNodes.push_back(neighbor);
                    PathNode* neighborPtr = &allNodes.back();

                    visited[coord] = newGCost;
                    neighborPtr->gCost = newGCost;
                    neighborPtr->hCost = calculateDistance3D(*neighborPtr, goalInternal);
                    neighborPtr->parent = current;

                    openSet.push(neighborPtr);
                }
            }
        }
    }

    return {}; 
}