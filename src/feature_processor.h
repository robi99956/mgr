#ifndef FEATURE_PROCESSOR_H
#define FEATURE_PROCESSOR_H

#include <opencv2/core/core.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using std::vector;

#include "matching_points.h"

enum class FeatureProcessorType {
    ORB, SIFT, KAZE, AKAZE
};

class FeatureProcessor {
    private:
        Ptr<Feature2D> detector;
        Ptr<DescriptorMatcher> matcher;
        int k = 2;
        float ratio_thresh = 0.7f;

        vector<DMatch> filter_matches(
                vector<vector<DMatch>> &matches, const float ratio_thresh);

        vector<MatchingPoints> connect_keypoints_and_set_center(
                vector<KeyPoint> &keypoints_prev, 
                vector<KeyPoint> &keypoints_current,
                vector<DMatch> &matches,
                Size image_size);

    public:
        FeatureProcessor(Ptr<Feature2D> detector, Ptr<DescriptorMatcher> matcher);
        FeatureProcessor(FeatureProcessorType type);

        void detect_and_compute(
                InputArray image, 
                vector<KeyPoint> &keypoints,
                OutputArray descriptors);

        vector<DMatch> knn_match(
                InputArray prev_desc, InputArray current_desc,
                int k, float ratio_thresh);

        vector<MatchingPoints> process(
                InputArray &previous_image, InputArray &current_image,
                bool display);
};
#endif
