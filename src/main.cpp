#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

#include <Eigen/Dense>

#include "feature_processor.h"
#include "optimization.h"

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

#if 1
class TestProblem : public OptimizationProblem {
    double f(int index, Vector &x) {
        switch( index ) {
            case 0: return atan(x[0]) - x[1] * x[1] * x[1];
            case 1: return 4*x[0] * x[0] + 9*x[1] * x[1] - 36;
        }

        return 0;
    }

    double df(int index, int derivative, Vector &x) {
        switch( index ) {
            case 0:
                switch( derivative ) {
                    case 0: return 1 / (1+x[0]*x[0]);
                    case 1: return -3*x[1]*x[1];
                }
                break;
            case 1:
                switch( derivative ) {
                    case 0: return 8*x[0];
                    case 1: return 18*x[1];
                }
        }

        return 0;
    }

    Vector weights( void ) {
        Vector w = Vector::Ones(2);
        return w;
    }
};

int main( void ) {
    TestProblem problem = TestProblem(); 
    Vector start(2);
    start[0] = 2.75;
    start[1] = 1.25;
    Vector X = newton_raphson_nonsquare(2, 2, &problem, start, 1e-6);

    cout << X << endl;
}
#endif
#if 0
int main( void ) {
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
#endif
#if 0
int main( void ) {
    srand(time(NULL));

   Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
   std::cout << "Here is the matrix A:\n" << A << std::endl;
   Eigen::VectorXf b = Eigen::VectorXf::Random(3);
   std::cout << "Here is the right hand side b:\n" << b << std::endl;

   std::cout << "The QR solution is:\n" <<
        A.colPivHouseholderQr().solve(b) << endl;
}
#endif
