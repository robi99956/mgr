#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

#include "feature_processor.h"

using namespace cv;
using std::cout;
using std::endl;

#define IMG_W 700
#define IMG_H ((IMG_W*16)/9)

#define PRINT_POINT(_p) printf(#_p" = [%f, %f]\n", _p.x, _p.y)

VideoCapture open_camera( void ) {
    return VideoCapture("test.mp4");
}

void print_connected_points( vector<MatchingPoints> &points ) {
    for( size_t i=0; i<points.size(); i++ ) {
        cout << points[i].to_string() << endl;
    }
}

int main( void )
{
    VideoCapture camera = open_camera();
    FeatureProcessor feature_processor(FeatureProcessorType::SIFT);

    Mat previous_image, current_image;
    camera >> previous_image;

    while( 1 ) {
        camera >> current_image;
        vector<MatchingPoints> connected_points = feature_processor.process(previous_image, current_image, true);
        print_connected_points(connected_points);

        previous_image = current_image;
        waitKey();
    }

    return 0;
}
