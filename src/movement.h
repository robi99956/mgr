#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "position.h"
#include "matrix_math.h"
#include "matching_points.h"

class MovementModel {
public:
    double dx, dy, dangle;
    double xo, yo;

    MovementModel( Vector &x );
    std::string to_string();
};

class MovementDetector {
private:
    VehiclePosition last_known_position;

public:
    MovementDetector();
    VehiclePosition process(std::vector<MatchingPoints> points);
};

#endif
