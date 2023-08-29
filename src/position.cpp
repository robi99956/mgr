#include "position.h"

VehiclePosition::VehiclePosition(double x, double y, double angle) {
    this->x = x;
    this->y = y;
    this->angle = angle;
}

VehiclePosition::VehiclePosition() {
    this->x = 0;
    this->y = 0;
    this->angle = 0;
}

std::string VehiclePosition::to_string() {
    char buf[128];
    sprintf(buf, "dx = %f, dy = %f, angle = %f",
            this->x, this->y, this->angle);
    return std::string(buf);
}
