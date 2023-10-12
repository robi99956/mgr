#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include "feature_processor.h"
#include "movement.h"
#include "optimization.h"
#include "image_preprocessing.h"

using namespace cv;
using std::cout;
using std::endl;

#define SCALE_FACTOR 1

#define PRINT_POINT(_p) printf(#_p" = [%f, %f]\n", _p.x, _p.y)

void print_connected_points( vector<MatchingPoints> &points ) {
    for( size_t i=0; i<points.size(); i++ ) {
        cout << points[i].to_string() << endl;
    }
}

#if 0
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

    int variable_count( void ) {
        return 2;
    }

    int row_count( void ) {
        return 2;
    }
};

int main( void ) {
    TestProblem problem = TestProblem(); 
    Vector start(2);
    start[0] = 2.75;
    start[1] = 1.25;
    Vector X = newton_raphson_nonsquare(&problem, start, 1e-6);

    cout << X << endl;
}
#endif
#if 1
void sharpen(Mat &img) {
    Mat kernel = (Mat_<double>(3,3) << 
            0, -1, 0,
            -1, 5, -1,
            0, -1, 0);
    filter2D(img, img, -1, kernel);
}

void remove_shutter_distortion(Mat &img, int speed) {
    Mat copy = img.clone(); 

    for( int y = 0; y<img.rows; y++ ) {
        int shift = (y*speed) / img.rows;
        for( int x=0; x<img.cols; x++ ) {
            int dst_x = x+shift;
            if( dst_x < 0 || dst_x > img.cols ) {
                continue;
            }
            img.at<uint8_t>(y, dst_x) = copy.at<uint8_t>(y, x);
        }
    }

    img = img(Rect(MAX(0, speed), 0, img.cols-2*speed, img.rows));
}

void preprocess_image(Mat &img) {
    resize(img, img, 
            Size(img.size[1] / SCALE_FACTOR, img.size[0] / SCALE_FACTOR));
    cvtColor(img, img, COLOR_BGR2GRAY);
//    GaussianBlur(img, img, Size(3, 3), 0);
    IAGCWD(img, img);
    sharpen(img);
//    remove_shutter_distortion(img, 50);
}

#define FRAME_SKIP_THRESHOLD 1

int main( int argc, char ** argv ) {
    (void)argc;
    std::string video_path = argv[1];
    VideoCapture camera(video_path, cv::CAP_ANY);
    if( camera.isOpened() == false ) {
        cout << "Camera open error" << endl;
        return 1;
    }
    FeatureProcessor feature_processor(FeatureProcessorType::ORB);
    MovementDetector movement_detector;

#if 1
//    Mat previous_image = imread("data/test7.jpg", IMREAD_GRAYSCALE);
//    Mat current_image = imread("data/test8.jpg", IMREAD_GRAYSCALE);
#else
    Mat previous_image = imread("images/test9.jpg", IMREAD_GRAYSCALE);
    Mat current_image = imread("images/test10.jpg", IMREAD_GRAYSCALE);
#endif

    Mat current_image, previous_image;
    camera >> previous_image;
    preprocess_image(previous_image);
    int dropped_updates = 0;

    int frame_counter = 0;
    while( camera.read(current_image) ) {
        preprocess_image(current_image);

        vector<MatchingPoints> connected_points = feature_processor.process(previous_image, current_image, true);
//        print_connected_points(connected_points);

        VehiclePositionSolution position_solution = movement_detector.process(connected_points);
        if( frame_counter % 50 == 0 ) {
            printf("%d,%f,%f,%f\n", 
                    frame_counter,
                    position_solution.position.x,
                    position_solution.position.y,
                    position_solution.position.angle);
        }
        frame_counter++;
//        cout << position_solution.position.to_string() << endl;
//        cout << position_solution.was_updated << endl;

        if( position_solution.was_updated || dropped_updates > FRAME_SKIP_THRESHOLD) {
            if( dropped_updates > FRAME_SKIP_THRESHOLD ) {
                cout << "Frame force updated" << endl;
            }
            cout << "Frame updated" << endl;
            dropped_updates = 0;
//            previous_image = current_image;
            current_image.copyTo(previous_image);
        } else {
            dropped_updates++;
        }
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
