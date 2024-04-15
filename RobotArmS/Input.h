#ifndef INPUT_H
#define INPUT_H
#include <iostream>
#include <limits>
#include <sstream>
#include <string>s
#include <Eigen/Dense>
#include "RobotClass.h"
#include <algorithm>
#include <cctype>
#include <locale>
struct Goal {
    Eigen::Vector4d position;
    Eigen::Matrix3d orientation;
};



// Trim from start (in place)
static inline void ltrim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
        }));
}

// Trim from end (in place)
static inline void rtrim(std::string& s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
        }).base(), s.end());
}

// Trim from both ends (in place)
static inline std::string trim(std::string& s) {
    ltrim(s);
    rtrim(s);
    return s;
}

bool getLineAsDoubles(double& roll_deg, double& pitch_deg, double& yaw_deg) {
    std::string line;
    std::getline(std::cin, line); // Read the entire line
    std::istringstream iss(line);
    iss >> roll_deg >> pitch_deg >> yaw_deg;
    return iss.eof() && !iss.fail(); // Check if we've read exactly three values and no errors
}
// Read a single line and convert to double, returning true if successful
bool getDoubleFromLine(double& value) {
    std::string line;
    std::getline(std::cin, line);  // Read the entire line
    if (line.empty()) return false;  // Allow default on empty line
    std::istringstream iss(line);
    iss >> value;
    return !iss.fail();  // True if successfully parsed a double
}

Goal getInput(RobotArm& arm) {
    std::cout << "Enter the desired position (x, y, z): ";
    double x, y, z;
    if (!(std::cin >> x >> y >> z)) {
        std::cin.clear();  // Clear error state
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Ignore bad input
        x = y = z = 0.0;  // Default position if input fails
    }

    std::cout << "Enter the desired orientation (roll, pitch, yaw) in degrees (leave blank for default 0, 0, 0): ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore leftover newline
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    getLineAsDoubles(roll_deg, pitch_deg, yaw_deg); // Only updates values if valid input is given

    // Convert degrees to radians
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    // Convert Euler angles to rotation matrix
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

    double alpha = arm.get_default_alpha();
    std::cout << "Enter desired alpha (press Enter for default): ";
    if (!getDoubleFromLine(alpha)) {  // If no input, keep default
        alpha = arm.get_default_alpha();
    }
    arm.set_alpha(alpha);

    double K = arm.get_default_K_orient();
    std::cout << "Enter desired K_orient (press Enter for default): ";
    if (!getDoubleFromLine(K)) {
        K = arm.get_default_K_orient();
    }
    arm.set_K_orient(K);

    return { Eigen::Vector4d(x, y, z, 1), q.matrix() };
}

#endif
