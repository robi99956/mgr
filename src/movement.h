#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "position.h"
#include "matching_points.h"

class MovementModel {
public:
    double dx, dy, dangle;
    double xo, yo;

    MovementModel(double dx, double dy, double dangle,
            double xo, double yo);
    MovementModel( void );
};

class MovementDetector {
private:
    VehiclePosition last_known_position;

public:
    VehiclePosition process(std::vector<MatchingPoints> points);
};

#endif
