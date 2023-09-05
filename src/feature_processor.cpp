#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"

#include "feature_processor.h"

vector<DMatch> FeatureProcessor::filter_matches(
        vector<vector<DMatch>> &matches, const float ratio_thresh) {

    //-- Filter matches using the Lowe's ratio test
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < ratio_thresh * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }
    return good_matches;
}

vector<MatchingPoints> FeatureProcessor::connect_keypoints_and_set_center(
        vector<KeyPoint> &keypoints_prev, 
        vector<KeyPoint> &keypoints_current,
        vector<DMatch> &matches,
        Size image_size) {

    Point2f center_offset(-image_size.width/2, -image_size.height/2);

    vector<MatchingPoints> connected_points;
    for( size_t i=0; i<matches.size(); i++ ) {
        Point2f point_prev = keypoints_prev[matches[i].queryIdx].pt + center_offset;
        Point2f point_current = keypoints_current[matches[i].trainIdx].pt + center_offset;
        connected_points.push_back(MatchingPoints(point_prev, point_current));
    }
    return connected_points;
}

FeatureProcessor::FeatureProcessor(
        Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher) {

    this->detector = detector;
    this->matcher = matcher;
}

FeatureProcessor::FeatureProcessor(FeatureProcessorType type) {
    switch( type ) {
        case FeatureProcessorType::ORB:
            this->detector = ORB::create(40); 
            this->matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
            break;
        case FeatureProcessorType::SIFT:
            this->detector = SIFT::create(40);
            this->matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            break;
        case FeatureProcessorType::KAZE:
            this->detector = KAZE::create(40);
            this->matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
            break;
        case FeatureProcessorType::AKAZE:
            this->detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB);
            this->matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
            break;
    }
}

void FeatureProcessor::detect_and_compute(
        InputArray image,
        vector<KeyPoint> &keypoints,
        OutputArray descriptors) {

    this->detector->detectAndCompute(image, noArray(), keypoints, descriptors);
}

vector<DMatch> FeatureProcessor::knn_match(
        InputArray prev_desc, InputArray current_desc,
        int k, float ratio_thresh) {

    vector<vector<DMatch>> knn_matches;
    try {
        this->matcher->knnMatch(prev_desc, current_desc, knn_matches, k); 
    } catch(...) {
        return vector<DMatch>();
    }
    return filter_matches(knn_matches, ratio_thresh);
}

vector<MatchingPoints> FeatureProcessor::process(
        InputArray &previous_image, InputArray &current_image,
        bool display) {

    vector<KeyPoint> keypoints_prev;
    Mat descriptors_prev;
    this->detect_and_compute(previous_image, keypoints_prev, descriptors_prev);

    vector<KeyPoint> keypoints_current;
    Mat descriptors_current;
    this->detect_and_compute(current_image, keypoints_current, descriptors_current);

    vector<DMatch> good_matches =
        this->knn_match(
                descriptors_prev, descriptors_current, this->k, this->ratio_thresh);

    if( display ) {
        Mat img_matches;
        drawMatches(previous_image, keypoints_prev, current_image, keypoints_current, 
                good_matches, img_matches, Scalar(0, 0, 255),
                Scalar::all(-1), 
                std::vector<char>(),
                DrawMatchesFlags::DEFAULT);
        imshow("Output", img_matches);
    }

    return this->connect_keypoints_and_set_center(
            keypoints_prev, keypoints_current, good_matches, previous_image.size());
}
