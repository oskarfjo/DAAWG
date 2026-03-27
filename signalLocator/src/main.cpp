#include <iostream>
#include <string>
#include <cmath>
#include <map>
#include <vector>
#include <numbers>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

const double EARTH_RADIUS_M = 6378137.0;

double toRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

struct Coordinate {
    double lat, lon;
    float alt;
    int rssi;
    int signalQuality;
};

Coordinate calculateSignalOrigin(double droneLat, double droneLon, double droneAlt, 
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

    std::vector<Coordinate> coordinateHistory;
    double dLat, dLon, dAlt, dHeading, sDist, sAngle;

    // Drone
    dLat = 62.2991464;
    dLon = 6.8483205;
    dAlt = 1; // (m) relative to ground
    dHeading = 0; // North = 0 deg
    sDist = 5; // meters

    int serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        std::cerr << "Error: Could not open serial port." << std::endl;
        return 1;
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) return 1;

    cfsetispeed(&tty, B9600); // Set Baud Rate
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem lines, enable receiver
    tty.c_cflag &= ~PARENB;         // No parity
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8 data bits

    tcsetattr(serial_port, TCSANOW, &tty);

    std::string angle_received;
    char c;
    bool capturing = false;

    while (true) {
        if (read(serial_port, &c, 1) > 0) {
            if (c == '#') {
                capturing = true;
            } else if (c == '&' && capturing) {
                capturing = false;
            } else if (capturing) {
                angle_received = c;

                if (angle_received == "0") {
                    sAngle = dHeading - 45;
                } else if (angle_received == "1") {
                    sAngle = dHeading - 22.5;
                } else if (angle_received == "2") {
                    sAngle = dHeading;
                } else if (angle_received == "3") {
                    sAngle = dHeading + 22.5;
                } else if (angle_received == "4") {
                    sAngle = dHeading + 45;
                } else {
                    std::cout << "bad angle_recieved = " << angle_received << std::endl;
                }
                
                Coordinate target = calculateSignalOrigin(dLat, dLon, dAlt, dHeading, sAngle, sDist);

                // TODO: consider calculating the signal quality using the SD of the rssi measurement

                coordinateHistory.push_back(target);

                if (coordinateHistory.size() > 20) { // the sensor currently gives values at a rate of ca 50 Hz
                    // TODO: Change the teensy code so that it only Serial.println when it gets a new ping. It currently prints the last value it got at a constant speed
                    coordinateHistory.erase(coordinateHistory.begin());
                }
                
                for (int i=0; i < coordinateHistory.size(); i++) {
                        std::cout << "Coordinate history: " << coordinateHistory[i].lat << ", " << coordinateHistory[i].lon << std::endl;
                }

                if (false) {
                    std::cout << std::fixed << std::setprecision(6);
                    std::cout << "\n--- Calculation Result ---\n";
                    std::cout << "Signal Origin: " << target.lat << ", " << target.lon << "\n";
                    std::cout << "Ground Distance: " << std::sqrt(std::pow(sDist, 2) - std::pow(dAlt, 2)) << " meters\n";
                    std::cout << "Parsed Message: " << angle_received << std::endl;
                }
            }
        }
    }

    close(serial_port);

    return 0;
}