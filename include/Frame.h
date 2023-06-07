//
// Created by bing on 2023/6/3.
//

#ifndef REPLACE_FRAME_H
#define REPLACE_FRAME_H

#include "vector"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "ORBextractor.h"
#include "MapPoint.h"
#include "Optimizer.h"

class ORBextractor;
class MapPoint;
class Optimizer;
class Frame {
public:
    Frame() {}
    // 用于加载BOW时使用
    Frame(const cv::Mat& colorImgs, const cv::Mat& depthImgs, const Eigen::Isometry3d pose, int i, cv::Mat K);
    // 重定位时读入帧
    Frame(const cv::Mat& colorImgs, int i, cv::Mat K);

//    // 通过深度图恢复出当前图像上2D特征点对应的3D地图点Pc（相机坐标系下）
//    void AddMapPoint(const Frame& CurF, const cv::Mat &K);
    // 通过深度图恢复出当前图像上2D特征点对应的3D地图点Pc（相机坐标系下）
    std::vector<cv::Point3d> GetMapPoints(std::shared_ptr<Frame> Bow_F);
//    std::vector<cv::Point2d> GetKeyPoints();

//    g2o::SE3Quat GetPose();
//    void SetPose();

public:
    std::shared_ptr<ORBextractor> orb_extractor = std::make_shared<ORBextractor>();
    std::vector<cv::KeyPoint> m_keypoints;   // 2D特征点
    cv::Mat m_des;              // 特征点描述子
    Eigen::Isometry3d m_pose;   // Twc
    cv::Mat m_colorImgs;
    cv::Mat m_depthImgs;

    int FrameId = 0; // 对应文件夹中的文件名，从1开始

    std::vector<cv::DMatch> m_orbmatches;

    cv::Mat m_K;

    std::vector<cv::Point3d> m_pts3d;  // 词带图像中的特征点对应的3D点（相机坐标系下）
    std::vector<cv::Point2d> m_pts2d;  // 当前图像中的2D点
    cv::Mat m_Tcr;   // 参考帧到当前帧的位姿变换矩阵(Bow_F 到 Cur_F)
};

#endif //REPLACE_FRAME_H
