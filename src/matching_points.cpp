#include <string>

#include "matching_points.h"

MatchingPoints::MatchingPoints(Point2f previous, Point2f current) {
    this->previous = previous;
    this->current = current;
}

std::string MatchingPoints::to_string( void ) {
    char buf[128];
    snprintf(buf, sizeof(buf),
            "[%f, %f] -> [%f, %f]", 
            previous.x, previous.y, current.x, current.y);
    return std::string(buf);
}

