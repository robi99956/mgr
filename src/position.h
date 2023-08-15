#ifndef POSITION_H
#define POSITION_H

#include <string>

class VehiclePosition {
public:
    double x;
    double y;
    double angle;

    VehiclePosition(double x, double y, double angle);
    VehiclePosition();

    std::string to_string();
};

#endif
