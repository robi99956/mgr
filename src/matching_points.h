#ifndef MATCHING_POINTS_H
#define MATCHING_POINTS_H

#include <opencv2/core/core.hpp>

using cv::Point2f;

class MatchingPoints {
    public:
        Point2f previous;
        Point2f current;

        MatchingPoints(Point2f previous, Point2f current);
        std::string to_string( void );
};


#endif
