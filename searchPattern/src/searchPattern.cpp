#include "searchPattern.h"
#include <cmath>
#include <vector>

using namespace CalculatedPath;

std::vector<Node> CalculatedPath::searchPattern(Node start, int length, int width, const MapData& map) {
    
    std::vector<Node> path; 

    int lengthNodes = std::ceil(static_cast<float>(length) / 10.0f);
    int widthNodes = std::ceil(static_cast<float>(width) / 10.0f);

    path.reserve((lengthNodes / 2) * 2 + 3);

    int topY = start.y + lengthNodes/2;
    int leftX = std::ceil(static_cast<float>(start.x) - static_cast<float>(widthNodes) / 2.0f);
    int rightX = std::ceil(static_cast<float>(start.x) + static_cast<float>(widthNodes) / 2.0f);

    // GENERATE NEXT
    bool goingLeft = true;
    for (int yOffset = 0; yOffset <= lengthNodes; yOffset += 2) {

        int currentY = topY - yOffset;

        Node firstNode;
        Node secondNode;

        firstNode.y = currentY;
        secondNode.y = currentY;

        if (goingLeft) {
            firstNode.x = leftX;
            secondNode.x = rightX;
        } else {
            firstNode.x = rightX;
            secondNode.x = leftX;
        }

        if (map.is_valid(firstNode.x, firstNode.y) && map.is_valid(secondNode.x, secondNode.y)) {
            firstNode.alt = map.get_elevation(firstNode.x, firstNode.y);
            secondNode.alt = map.get_elevation(secondNode.x, secondNode.y);
            path.push_back(firstNode);
            path.push_back(secondNode);
        } else {
            return {};
        }

        goingLeft =! goingLeft;
    }

    return path;
}