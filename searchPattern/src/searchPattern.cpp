#include "searchPattern.h"
#include <cmath>
#include <vector>

using namespace CalculatedPath;

std::vector<Node> CalculatedPath::searchPattern(Node center, int length, int width, const MapData& map) {
    
    std::vector<Node> path; 

    // Ensure dimentionality of the lenght and width (DOM10)
    int lengthNodes = std::ceil(static_cast<float>(length) / 10.0f);
    int widthNodes = std::ceil(static_cast<float>(width) / 10.0f);

    path.reserve((lengthNodes / 2) * 2 + 3);

    int sampleOffset = widthNodes/2;
    float dz_dx = map.get_elevation(center.x + sampleOffset, center.y) - map.get_elevation(center.x - sampleOffset, center.y);
    float dz_dy = map.get_elevation(center.x, center.y + sampleOffset) - map.get_elevation(center.x, center.y - sampleOffset);

    // Defines roation angle to orient the y-axis with the gradient of the slope
    float theta = std::atan2(dz_dy, dz_dx);

    float sinT = std::sin(theta);
    float cosT = std::cos(theta);

    // Defines bounds (local to center)
    int topY    = lengthNodes / 2;
    int bottomY = -lengthNodes / 2;
    int leftX   = -widthNodes / 2;
    int rightX  = widthNodes / 2;

    // Adds a layer of nodes iteratively down the defined area starting from the top
    bool goingLeft = true;
    for (int currentY = topY; currentY >= bottomY; currentY -= 2) {

        Node firstNode;
        Node secondNode;

        int firstY = currentY;
        int secondY = currentY;
        int firstX, secondX;

        if (goingLeft) {
            firstX = leftX;
            secondX = rightX;
        } else {
            firstX = rightX;
            secondX = leftX;
        }

        // Rotation matrixes : theta=0 is paralell to global Y
        // rot_x = x * |sin(ø)  cos(ø)| 
        //             |-cos(ø) sin(ø)|

        float rotatedFirstX = firstX * sinT + firstY * cosT;
        float rotatedFirstY = firstX * (-cosT) + firstY * sinT;
        firstNode.x = std::round(center.x + rotatedFirstX);
        firstNode.y = std::round(center.y + rotatedFirstY);

        float rotatedSecondX = secondX * sinT + secondY * cosT;
        float rotatedSecondY = secondX * (-cosT) + secondY * sinT;
        secondNode.x = std::round(center.x + rotatedSecondX);
        secondNode.y = std::round(center.y + rotatedSecondY);

        // Adds the nodes to the path vector
        if (map.is_valid(firstNode.x, firstNode.y) && map.is_valid(secondNode.x, secondNode.y)) {
            firstNode.alt = map.get_elevation(firstNode.x, firstNode.y);
            secondNode.alt = map.get_elevation(secondNode.x, secondNode.y);
            path.push_back(firstNode);
            path.push_back(secondNode);
        } else {
            return {};
        }

        goingLeft = !goingLeft;
    }

    return path;
}