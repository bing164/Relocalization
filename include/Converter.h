//
// Created by bing on 2023/6/5.
//

#ifndef REPLACE_CONVERTER_H
#define REPLACE_CONVERTER_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include"../Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

class Converter {
public:
    Converter() {}

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
};

#endif //REPLACE_CONVERTER_H
