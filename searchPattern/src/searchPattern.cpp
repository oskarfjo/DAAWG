#include "searchPattern.h"
#include <cmath>
#include <vector>
#include <iostream>

using namespace CalculatedPath;

const double EARTH_RADIUS_M = 6378137.0;

static double toRadians(double degrees) { return degrees * (M_PI / 180.0); }
static double toDegrees(double radians) { return radians * (180.0 / M_PI); }

double normalizeDeg(double angle) {
    angle = std::fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;
    return angle;
}

Waypoint calculateDestinationCoord(Waypoint start, double angle, double distance) {
    double lat1 = toRadians(start.lat);
    double lon1 = toRadians(start.lon);
    double angularDist = distance / EARTH_RADIUS_M;
    double bearingRad = toRadians(normalizeDeg(angle));

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
    double halfWidth = width / 2.0;
    double halfLength = length / 2.0;

    Waypoint roughNorthPt = calculateDestinationCoord(center, 0.0, halfLength);
    Waypoint roughSouthPt = calculateDestinationCoord(center, 180.0, halfLength);
    Waypoint roughEastPt  = calculateDestinationCoord(center, 90.0, halfWidth);
    Waypoint roughWestPt  = calculateDestinationCoord(center, 270.0, halfWidth);

    float roughAltN = loader.get_elevation_at_coordinate(map, roughNorthPt.lat, roughNorthPt.lon);
    float roughAltS = loader.get_elevation_at_coordinate(map, roughSouthPt.lat, roughSouthPt.lon);
    float roughAltE = loader.get_elevation_at_coordinate(map, roughEastPt.lat, roughEastPt.lon);
    float roughAltW = loader.get_elevation_at_coordinate(map, roughWestPt.lat, roughWestPt.lon);

    if (roughAltN < -9999 || roughAltS < -9999 || roughAltE < -9999 || roughAltW < -9999) {
        std::cerr << "Error: Center point too close to map edge to calculate gradient." << std::endl;
        return {};
    }
    
    double rough_dz_dN = (roughAltN - roughAltS) / static_cast<double>(length);
    double rough_dz_dE = (roughAltE - roughAltW) / static_cast<double>(width);

    double roughGradientRad = std::atan2(rough_dz_dN, rough_dz_dE);
    double roughGradientDeg = 90 - toDegrees(roughGradientRad);
    
    roughGradientDeg = normalizeDeg(roughGradientDeg);

    std::cout << "rough gradient: " << roughGradientDeg << "°" << std::endl;

    Waypoint refinedNorthPt = calculateDestinationCoord(center, roughGradientDeg, halfLength);
    Waypoint refinedSouthPt = calculateDestinationCoord(center, roughGradientDeg + 180.0, halfLength);
    Waypoint refinedEastPt  = calculateDestinationCoord(center, roughGradientDeg + 90.0, halfWidth);
    Waypoint refinedWestPt  = calculateDestinationCoord(center, roughGradientDeg + 270.0, halfWidth);

    float refinedAltN = loader.get_elevation_at_coordinate(map, refinedNorthPt.lat, refinedNorthPt.lon);
    float refinedAltS = loader.get_elevation_at_coordinate(map, refinedSouthPt.lat, refinedSouthPt.lon);
    float refinedAltE = loader.get_elevation_at_coordinate(map, refinedEastPt.lat, refinedEastPt.lon);
    float refinedAltW = loader.get_elevation_at_coordinate(map, refinedWestPt.lat, refinedWestPt.lon);

    if (refinedAltN < -9999 || refinedAltS < -9999 || refinedAltE < -9999 || refinedAltW < -9999) {
        std::cerr << "Error: Center point too close to map edge to calculate gradient." << std::endl;
        return {};
    }
    
    double refined_dz_dN = (refinedAltN - refinedAltS) / static_cast<double>(length);
    double refined_dz_dE = (refinedAltE - refinedAltW) / static_cast<double>(width);

    double refinedGradientRad = std::atan2(refined_dz_dN, refined_dz_dE);
    double refinedGradientDeg = roughGradientDeg - (90 - toDegrees(refinedGradientRad));
    
    refinedGradientDeg = normalizeDeg(refinedGradientDeg);

    std::cout << "refined gradient: " << refinedGradientDeg << "°" << std::endl;

    // Local coordinate system
    double yAxisBearing = refinedGradientDeg;
    double xAxisBearing = refinedGradientDeg + 90.0;

    double laneSpacing = 20; // (m)

    Waypoint topPt = calculateDestinationCoord(center, refinedGradientDeg, halfLength);
    Waypoint bottomPt = calculateDestinationCoord(center, refinedGradientDeg + 180.0, halfLength);

    topPt.alt = loader.get_elevation_at_coordinate(map, topPt.lat, topPt.lon);
    bottomPt.alt = loader.get_elevation_at_coordinate(map, bottomPt.lat, bottomPt.lon);

    std::cout << "Top pt alt: " << topPt.alt << "moh" << std::endl;
    std::cout << "Bottom pt alt: " << bottomPt.alt << "moh" << std::endl;

    double dz_dy = (topPt.alt - bottomPt.alt) / static_cast<double>(length);
    double slopeAngle = std::atan(dz_dy);
    double projectedLaneSpacing = laneSpacing * std::cos(slopeAngle);

    std::cout << "dz_dy: " << dz_dy << std::endl;
    std::cout << "Projected angle: " << slopeAngle*180/M_PI << "°" << std::endl;
    std::cout << "Projected spacing: " << projectedLaneSpacing << "m" << std::endl;

    bool goingRight = true;
    for (double y = halfLength; y >= -halfLength; y -= projectedLaneSpacing) {
        
        double xStart, xEnd;
        
        if (goingRight) {
            xStart  = halfWidth;
            xEnd    = -halfWidth;
        } else {
            xStart  = -halfWidth;
            xEnd    = halfWidth;
        }

        Waypoint wpA = center; 
        wpA = calculateDestinationCoord(wpA, yAxisBearing, y);
        wpA = calculateDestinationCoord(wpA, xAxisBearing, xStart);
        
        Waypoint wpB = center;
        wpB = calculateDestinationCoord(wpB, yAxisBearing, y);
        wpB = calculateDestinationCoord(wpB, xAxisBearing, xEnd);

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