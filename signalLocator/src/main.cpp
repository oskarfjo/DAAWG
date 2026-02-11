#include <iostream>
#include <string>
#include <cmath>
#include <map>
#include <numbers>
#include <iomanip>

const double EARTH_RADIUS_M = 6378137.0;

double toRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

struct Coordinates {
    double lat;
    double lon;
};

Coordinates calculateSignalOrigin(double droneLat, double droneLon, double droneAlt, 
                                  double droneHeading, double sensorAngle, double sensorDistance) {

    // https://www.movable-type.co.uk/scripts/latlong.html
    
    double groundDistance = 0.0;
    if (sensorDistance > droneAlt) {
        groundDistance = std::sqrt(std::pow(sensorDistance, 2) - std::pow(droneAlt, 2));
    } else {
        groundDistance = 0.0; 
    }

    double absoluteBearingDeg = droneHeading + sensorAngle;
    double bearingRad = toRadians(absoluteBearingDeg);

    double lat1 = toRadians(droneLat);
    double lon1 = toRadians(droneLon);
    double angularDist = groundDistance / EARTH_RADIUS_M;

    double lat2 = std::asin(std::sin(lat1) * std::cos(angularDist) +
                            std::cos(lat1) * std::sin(angularDist) * std::cos(bearingRad));

    double lon2 = lon1 + std::atan2(std::sin(bearingRad) * std::sin(angularDist) * std::cos(lat1),
                                    std::cos(angularDist) - std::sin(lat1) * std::sin(lat2));

    return {toDegrees(lat2), toDegrees(lon2)};
}

int main() {
    double dLat, dLon, dAlt, dHeading, sDist, sAngle;

    // Drone
    dLat = 62.2991464;
    dLon = 6.8483205;
    dAlt = 25; // (m) relative to ground
    dHeading = 0; // North = 0 deg

    // Sensor
    sDist = 40;
    sAngle = 45;

    Coordinates target = calculateSignalOrigin(dLat, dLon, dAlt, dHeading, sAngle, sDist);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n--- Calculation Result ---\n";
    std::cout << "Signal Origin: " << target.lat << ", " << target.lon << "\n";
    std::cout << "Ground Distance: " << std::sqrt(std::pow(sDist, 2) - std::pow(dAlt, 2)) << " meters\n";

    return 0;
}