//
// Created by bing on 2023/6/3.
//

#include "Frame.h"

Frame::Frame(std::shared_ptr<Frame> cur) :
m_colorImgs(cur->m_colorImgs.clone()), m_keypoints(cur->m_keypoints), m_des(cur->m_des.clone())
{

}

Frame::Frame(const cv::Mat &colorImgs, const cv::Mat &depthImgs, const Eigen::Isometry3d pose, int i, cv::Mat K)
    : m_colorImgs(colorImgs), m_depthImgs(depthImgs), m_pose(pose), m_K(K) {
    orb_extractor->ExtractORB(colorImgs, m_keypoints, m_des);
    FrameId = i;
}

Frame::Frame(const cv::Mat &colorImgs, int i, cv::Mat K) : m_colorImgs(colorImgs), m_K(K){
    orb_extractor->ExtractORB(colorImgs, m_keypoints, m_des);
    FrameId = i;
}

inline cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d(
            (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
            (p.y - K.at<double>(1,2)) / K.at<double>(1,1)
            );
}

//void Frame::AddMapPoint(const Frame &CurF, const cv::Mat &K) {
//    for (auto m : CurF.m_orbmatches) {
//        ushort d = CurF.m_depthImgs.ptr<unsigned short>(int(CurF.m_keypoints[m.queryIdx].pt.y))[int(CurF.m_keypoints[m.queryIdx].pt.x)];
//        if (d == 0)
//            continue;
//
//        float dd = d / 5000;
//        cv::Point2d p1 = pixel2cam(CurF.m_keypoints[m.queryIdx].pt, K);
//        m_pts3d.push_back(cv::Point3d(p1.x * dd, p1.y * dd, dd));
////        m_pts2d.push_back(cv::Point2d(CurF.m_keypoints[]))
//    }
//}

std::vector<cv::Point3d> Frame::GetMapPoints(std::shared_ptr<Frame> Bow_F) {
//    std::cout << "m_orbmatches1111 = " << Bow_F->m_orbmatches.size() << std::endl;
    for (auto m : Bow_F->m_orbmatches) {
        unsigned int d = Bow_F->m_depthImgs.ptr<unsigned short>(int(Bow_F->m_keypoints[m.queryIdx].pt.y))[int(Bow_F->m_keypoints[m.queryIdx].pt.x)];
//        std::cout << "d = " << d << std::endl;
        if (d == 0)
            continue;

        float dd = d / 5000.0;
//        std::cout << "dd = " << dd << std::endl;
        cv::Point2d p1 = pixel2cam(Bow_F->m_keypoints[m.queryIdx].pt, Bow_F->m_K);
        Bow_F->m_pts3d.push_back(cv::Point3d(p1.x * dd, p1.y * dd, dd));
        this->m_pts2d.push_back(this->m_keypoints[m.trainIdx].pt);
    }
//    std::cout << "11 = " << Bow_F->m_pts3d.size() << "\n 222 = " << this->m_pts2d.size() << std::endl;
    return m_pts3d;
}

//std::vector<cv::Point2d> Frame::GetKeyPoints() {
//    std::cout << "m_orbmatches2222 = " << this->m_orbmatches.size() << std::endl;
//    for (auto m : this->m_orbmatches) {
//        m_pts2d.push_back(this->m_keypoints[m.trainIdx].pt);
//    }
//    return m_pts2d;
//}

