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

#define IMG_W 700
#define IMG_H ((IMG_W*16)/9)

#define PRINT_POINT(_p) printf(#_p" = [%f, %f]\n", _p.x, _p.y)

VideoCapture open_camera( void ) {
    return VideoCapture("test.mp4");
}

// 1. Init pobierajki obrazu
// 2. Init detektora(ów)?
// 3. while(1)
// 4. detectAndCompute
// 5. match
// 6. weryfikacja czy git
// 7. obliczenie przesunięcia i obrotu
// 8. weryfikacja teleportacji

int main( void )
{
//    Mat img1 = imread("images/test3.png", IMREAD_GRAYSCALE);
//    Mat img2 = imread("images/test4.png", IMREAD_GRAYSCALE);
//    Mat img1 = imread("images/test1.png", IMREAD_GRAYSCALE);
//    Mat img2 = imread("images/test2.png", IMREAD_GRAYSCALE);
    Mat img1 = imread("images/test7.jpg", IMREAD_GRAYSCALE);
    Mat img2 = imread("images/test8.jpg", IMREAD_GRAYSCALE);
    resize(img1, img1, Size(IMG_W, IMG_H));
    resize(img2, img2, Size(IMG_W, IMG_H));

//    int8_t kernel_data[] = {0, -1, 0, -1, 5, -1, 0, -1, 0};
//    Mat kernel = Mat(Size(3, 3), CV_8S, kernel_data);
//
//    filter2D(img2, img2, 0, kernel);
//
//    Rect roi1 = Rect(0, 0, 400, 400);
//    Rect roi2 = Rect(50, 50, 400, 400);
//
//    img1 = Mat(img2, roi1);
//    img2 = Mat(img2, roi2);

    // *** ORB
//    Ptr<ORB> detector = ORB::create(40);
//    Ptr<DescriptorMatcher> matcher =
//        DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    // *** SIFT
    Ptr<SIFT> detector = SIFT::create(40);
    Ptr<DescriptorMatcher> matcher =
        DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    // *** KAZE
//    Ptr<KAZE> detector = KAZE::create();
//    Ptr<DescriptorMatcher> matcher =
//        DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    // *** AKAZE
//    Ptr<AKAZE> detector = AKAZE::create();
//    Ptr<DescriptorMatcher> matcher =
//        DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

    std::vector<KeyPoint> keypoints_1;
    Mat descriptors1;
    detector->detectAndCompute(img1, noArray(), keypoints_1, descriptors1);
    std::vector<KeyPoint> keypoints_2;
    Mat descriptors2;
    detector->detectAndCompute(img2, noArray(), keypoints_2, descriptors2);

    printf("dsc1 = %d, dsc2 = %d\n", (int)keypoints_1.size(), (int)keypoints_2.size());

    std::vector<std::vector<DMatch>> knn_matches;

    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    printf("good.size() = %d\n", (int)good_matches.size());

    for( size_t i=0; i<good_matches.size(); i++ ) {
        Point2f point_1 = keypoints_2[good_matches[i].trainIdx].pt;
        Point2f point_2 = keypoints_1[good_matches[i].queryIdx].pt;
        Point2f diff = point_2 - point_1;
        PRINT_POINT(diff);
    }

    Mat img_matches;
    drawMatches(img1, keypoints_1, img2, keypoints_2, 
            good_matches, img_matches, Scalar(0, 0, 255),
            Scalar::all(-1), 
            std::vector<char>(),
            DrawMatchesFlags::DEFAULT);
    imshow("Output", img_matches);

    waitKey();
    return 0;
}
