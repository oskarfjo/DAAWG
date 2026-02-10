#include "searchPattern.h"
#include <cmath>
#include <vector>
#include <iostream>

using namespace CalculatedPath;

const double EARTH_RADIUS_M = 6378137.0;

static double toRadians(double degrees) { return degrees * (M_PI / 180.0); }
static double toDegrees(double radians) { return radians * (180.0 / M_PI); }

// Calculates a destination coordinate given a start point, bearing (degrees), and distance (meters)
Waypoint calculateDestination(Waypoint start, double bearingDeg, double distanceMeters) {
    double lat1 = toRadians(start.lat);
    double lon1 = toRadians(start.lon);
    double angularDist = distanceMeters / EARTH_RADIUS_M;
    double bearingRad = toRadians(bearingDeg);

    double lat2 = std::asin(std::sin(lat1) * std::cos(angularDist) +
                            std::cos(lat1) * std::sin(angularDist) * std::cos(bearingRad));

    double lon2 = lon1 + std::atan2(std::sin(bearingRad) * std::sin(angularDist) * std::cos(lat1),
                                    std::cos(angularDist) - std::sin(lat1) * std::sin(lat2));

    Waypoint wp;
    wp.lat = toDegrees(lat2);
    wp.lon = toDegrees(lon2);
    return wp;
}

std::vector<Waypoint> CalculatedPath::searchPattern(Waypoint center, int length, int width, const MapData& map, const GeoLoader& loader) {
    std::vector<Waypoint> path;

    // 1. Calculate Gradient
    double sampleDist = static_cast<float>(width) / 2.0f;

    Waypoint northPt = calculateDestination(center, 0.0, sampleDist);
    Waypoint southPt = calculateDestination(center, 180.0, sampleDist);
    Waypoint eastPt  = calculateDestination(center, 90.0, sampleDist);
    Waypoint westPt  = calculateDestination(center, 270.0, sampleDist);

    float hN = loader.get_elevation_at_coordinate(map, northPt.lat, northPt.lon);
    float hS = loader.get_elevation_at_coordinate(map, southPt.lat, southPt.lon);
    float hE = loader.get_elevation_at_coordinate(map, eastPt.lat, eastPt.lon);
    float hW = loader.get_elevation_at_coordinate(map, westPt.lat, westPt.lon);

    if (hN < -9000 || hS < -9000 || hE < -9000 || hW < -9000) {
        std::cerr << "Error: Center point too close to map edge to calculate gradient." << std::endl;
        return {};
    }

    double dz_dy = (hN - hS) / (2 * sampleDist); // Change in height per meter North
    double dz_dx = (hE - hW) / (2 * sampleDist); // Change in height per meter East

    // Calculate aspect (direction of slope). 
    // atan2(y, x) gives angle from X-axis. We want compass bearing.
    // Math angle 0 is East. Compass 0 is North.
    // Compass Bearing = 90 - MathAngle
    double aspectRad = std::atan2(dz_dy, dz_dx); // Math angle
    double aspectDeg = toDegrees(aspectRad);
    
    // Convert math angle to compass bearing for "Down Slope"
    // The gradient vector points UPHILL. We usually want to search DOWN or ACROSS.
    // Let's assume we align the "Length" of the search box with the gradient (up/down).
    double gradientBearing = aspectDeg; 
    
    // Normalize
    if (gradientBearing < 0) gradientBearing += 360.0;

    // Angles of local coordinate system
    double yAxisBearing = gradientBearing;     // Align with slope
    double xAxisBearing = yAxisBearing + 90.0; // Perpendicular to slope

    int laneSpacing = 20; // (m)
    int numLanes = std::ceil((float)width / laneSpacing);
    
    double halfWidth = width / 2.0;
    double halfLength = length / 2.0;

    bool goingRight = true;
    for (double y = halfLength; y >= -halfLength; y -= laneSpacing) {
        
        double xStart, xEnd;
        
        if (goingRight) {
            xStart = halfWidth;
            xEnd = -halfWidth;
        } else {
            xStart = -halfWidth;
            xEnd = halfWidth;
        }

        Waypoint wpA = center; 
        wpA = calculateDestination(wpA, xAxisBearing, xStart);
        wpA = calculateDestination(wpA, yAxisBearing, y);
        
        Waypoint wpB = center;
        wpB = calculateDestination(wpB, xAxisBearing, xEnd);
        wpB = calculateDestination(wpB, yAxisBearing, y);

        wpA.alt = loader.get_elevation_at_coordinate(map, wpA.lat, wpA.lon) + 15;
        wpB.alt = loader.get_elevation_at_coordinate(map, wpB.lat, wpB.lon) + 15;

        if (wpA.alt < -9000 || wpB.alt < -9000) {
            std::cerr << "Warning: Generated waypoint outside map bounds." << std::endl;
        } else {
            path.push_back(wpA);
            path.push_back(wpB);
        }

        goingRight = !goingRight;
    }

    return path;
}