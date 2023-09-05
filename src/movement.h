#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "position.h"
#include "matrix_math.h"
#include "matching_points.h"

class VehiclePositionSolution {
    public:
        VehiclePosition position;
        bool was_updated;
        VehiclePositionSolution(VehiclePosition position, bool was_updated) {
            this->position = position;
            this->was_updated = was_updated;
        }
};

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
    VehiclePositionSolution process(std::vector<MatchingPoints> points);
};

#endif
