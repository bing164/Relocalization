//
// Created by bing on 2023/6/3.
//

#include "ORBextractor.h"

ORBextractor::ORBextractor() {
    orb_ = cv::ORB::create(2000);
}

int ORBextractor::ExtractORB(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &des) {
    if (orb_.empty()) {
        std::cout << "orb_ is empty!!!!" << std::endl;
        return -1;
    }
    orb_->detectAndCompute(image, cv::Mat(), keypoints, des);
    return keypoints.size();
}
