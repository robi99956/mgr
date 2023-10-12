#ifndef IMAGE_PREPROCESSING_H
#define IMAGE_PREPROCESSING_H

#include <iostream>
#include <opencv2/opencv.hpp>

/***
@article{cao2017contrast,
  title={Contrast enhancement of brightness-distorted images by improved adaptive gamma correction},
  author={Cao, Gang and Huang, Lihui and Tian, Huawei and Huang, Xianglin and Wang, Yongbin and Zhi, Ruicong},
  journal={Computers & Electrical Engineering},
  volume={66},
  pages={569--582},
  year={2017}
}
***/
void IAGCWD(const cv::Mat & src, cv::Mat & dst,
        double alpha_dimmed = 0.75, double alpha_bright = 0.25,
        int T_t = 112, double tau_t = 0.3, double tau = 0.5);


#endif
