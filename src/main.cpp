#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
using std::vector;

#define IMG_W 700
#define IMG_H ((IMG_W*16)/9)

#define PRINT_POINT(_p) printf(#_p" = [%f, %f]\n", _p.x, _p.y)

VideoCapture open_camera( void ) {
    return VideoCapture("test.mp4");
}

enum class FeatureProcessorType {
    ORB, SIFT, KAZE, AKAZE
};

class MatchingPoints {
    public:
        Point2f previous;
        Point2f current;

        MatchingPoints(Point2f previous, Point2f current) {
            this->previous = previous;
            this->current = current;
        }

        std::string to_string( void ) {
            char buf[128];
            snprintf(buf, sizeof(buf),"[%f, %f] -> [%f, %f]", previous.x, previous.y, current.x, current.y);
            return String(buf);
        }
};

class FeatureProcessor {
    private:
        Ptr<Feature2D> detector;
        Ptr<DescriptorMatcher> matcher;
        int k = 2;
        float ratio_thresh = 0.7f;

        vector<DMatch> filter_matches(vector<vector<DMatch>> &matches, const float ratio_thresh) {
            //-- Filter matches using the Lowe's ratio test
            std::vector<DMatch> good_matches;
            for (size_t i = 0; i < matches.size(); i++) {
                if (matches[i][0].distance < ratio_thresh * matches[i][1].distance) {
                    good_matches.push_back(matches[i][0]);
                }
            }
            return good_matches;
        }

        vector<MatchingPoints> connect_keypoints(vector<KeyPoint> &keypoints_prev, vector<KeyPoint> &keypoints_current, vector<DMatch> &matches) {
            vector<MatchingPoints> connected_points;
            for( size_t i=0; i<matches.size(); i++ ) {
                Point2f point_prev = keypoints_prev[matches[i].trainIdx].pt;
                Point2f point_current = keypoints_current[matches[i].queryIdx].pt;
                connected_points.push_back(MatchingPoints(point_prev, point_current));
            }
            return connected_points;
        }

    public:
        FeatureProcessor(Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher) {
            this->detector = detector;
            this->matcher = matcher;
        }

        FeatureProcessor(FeatureProcessorType type) {
            switch( type ) {
                case FeatureProcessorType::ORB:
                    FeatureProcessor(ORB::create(40), 
                            DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING));
                    break;
                case FeatureProcessorType::SIFT:
                    FeatureProcessor(SIFT::create(40), 
                            DescriptorMatcher::create(DescriptorMatcher::FLANNBASED));
                    break;
                case FeatureProcessorType::KAZE:
                    FeatureProcessor(KAZE::create(40), 
                            DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING));
                    break;
                case FeatureProcessorType::AKAZE:
                    FeatureProcessor(AKAZE::create(AKAZE::DESCRIPTOR_MLDB), 
                            DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING));
                    break;
            }
        }

        void detect_and_compute(InputArray image, vector<KeyPoint> &keypoints, OutputArray descriptors) {
            this->detector->detectAndCompute(image, noArray(), keypoints, descriptors);
        }

        vector<DMatch> knn_match(InputArray prev_desc, InputArray current_desc, int k, float ratio_thresh) {
            vector<vector<DMatch>> knn_matches;
            this->matcher->knnMatch(current_desc, prev_desc, knn_matches, k); 
            return filter_matches(knn_matches, ratio_thresh);
        }

        vector<MatchingPoints> process(InputArray previous_image, InputArray current_image, bool display) {
            vector<KeyPoint> keypoints_prev;
            Mat descriptors_prev;
            this->detect_and_compute(previous_image, keypoints_prev, descriptors_prev);

            vector<KeyPoint> keypoints_current;
            Mat descriptors_current;
            this->detect_and_compute(current_image, keypoints_current, descriptors_current);

            vector<DMatch> good_matches = this->knn_match(descriptors_prev, descriptors_current, this->k, this->ratio_thresh);
            if( display ) {
                Mat img_matches;
                drawMatches(previous_image, keypoints_prev, current_image, keypoints_current, 
                        good_matches, img_matches, Scalar(0, 0, 255),
                        Scalar::all(-1), 
                        std::vector<char>(),
                        DrawMatchesFlags::DEFAULT);
                imshow("Output", img_matches);
            }

            return this->connect_keypoints(keypoints_prev, keypoints_current, good_matches);
        }
};

// 1. Init pobierajki obrazu
// 2. Init detektora(ów)?
// 3. while(1)
// 4. detectAndCompute
// 5. match
// 6. weryfikacja czy git
// 7. obliczenie przesunięcia i obrotu
// 8. weryfikacja teleportacji

void print_connected_points( vector<MatchingPoints> &points ) {
    for( size_t i=0; i<points.size(); i++ ) {
        cout << points[i].to_string() << endl;
    }
}

int main( void )
{
    VideoCapture camera = open_camera();
    FeatureProcessor feature_processor(FeatureProcessorType::ORB);

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
