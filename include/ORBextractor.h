//
// Created by bing on 2023/6/3.
//

#ifndef REPLACE_ORBEXTRACTOR_H
#define REPLACE_ORBEXTRACTOR_H

#include "opencv2/opencv.hpp"

class ORBextractor {
public:
    ORBextractor();
    ~ORBextractor() {}

    int ExtractORB(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& des);

private:
    cv::Ptr<cv::ORB> orb_;
};


#endif //REPLACE_ORBEXTRACTOR_H
