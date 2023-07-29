#include "movement.h"

MovementModel::MovementModel(double dx, double dy, double dangle,
            double xo, double yo) {
    this->dx = dx;
    this->dy = dy;
    this->dangle = dangle;
    this->xo = xo;
    this->yo = yo;
}

MovementModel::MovementModel( void ) {
    this->dx = 0;
    this->dy = 0;
    this->dangle = 0;
    this->xo = 0;
    this->yo = 0;
}

VehiclePosition MovementDetector::process(
        std::vector<MatchingPoints> points) {

    (void)points;
    return VehiclePosition(0, 0, 0);
}
