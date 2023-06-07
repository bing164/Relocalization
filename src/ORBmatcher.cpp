//
// Created by bing on 2023/6/3.
//

#include "ORBmatcher.h"

ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : m_nnratio(nnratio), m_checkOri(checkOri) {

}

int ORBmatcher::SearchForReplace(const std::string &img_path, std::shared_ptr<Frame> Bow_F, std::shared_ptr<Frame> Cur_F) {
//    std::cout << "./data/" + std::to_string(Bow_F.FrameId) + ".png" << std::endl;
//    std::cout << "./data/" + std::to_string(Cur_F.FrameId) + ".png" << std::endl;
//    cv::Mat img1 = cv::imread(img_path + "color/" + std::to_string(Bow_F->FrameId) + ".png");
//    cv::Mat img2 = cv::imread(img_path + "color/" + std::to_string(1) + ".png");
    cv::Mat img1 = Bow_F->m_colorImgs;
    cv::Mat img2 = Cur_F->m_colorImgs;

    // 使用汉明距离计算特征点描述子之间的距离
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(Bow_F->m_des, Cur_F->m_des, matches, 2);



    // 进行比值测试，保留最近匹配点与次近匹配点的距离比小于一定阈值的匹配点
    const float ratio_thresh = 0.75f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < ratio_thresh * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    // 绘制最终的匹配结果
//    cv::Mat match_image0;
//    cv::drawMatches(img1, Bow_F->m_keypoints, img2, Cur_F->m_keypoints, good_matches, match_image0,
//                    cv::Scalar::all(-1),  cv::Scalar::all(-1), std::vector<char>(),
//                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//    std::cout << "good matches size = " << good_matches.size() << std::endl;

    // 使用RANSAC算法剔除错误匹配
    std::vector<cv::Point2f> points1, points2;
    for (size_t i = 0; i < good_matches.size(); i++) {
        points1.push_back(Bow_F->m_keypoints[good_matches[i].queryIdx].pt);
        points2.push_back(Cur_F->m_keypoints[good_matches[i].trainIdx].pt);
    }
    std::vector<uchar> inliers(points1.size());
    cv::Mat fundamental_matrix = cv::findFundamentalMat(points1, points2, inliers, cv::FM_RANSAC);

    // cv::Mat K = (cv::Mat_<double>(3,3) << 481.2, 0, 319.5, 0, -480.0, 239.5, 0, 0, 1);
//    cv::Mat K = (cv::Mat_<double>(3,3) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1);
//    cv::Point2d principal_point(319.5, 239.5);  //相机光心, TUM dataset标定值
//    double focal_length = 480;      //相机焦距, TUM dataset标定值
//    cv::Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
//
//    cv::Mat R, t;
//    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
//    std::cout << "p2p R = \n" << R << std::endl;
//    std::cout << "p2p t = \n" << t << std::endl;
    // 保留RANSAC筛选出的内点匹配点
    std::vector<cv::DMatch> ransac_matches;
    for (size_t i = 0; i < good_matches.size(); i++) {
        if (inliers[i]) {
            ransac_matches.push_back(good_matches[i]);
        }
    }

    // 绘制最终的匹配结果
//    cv::Mat match_image;
//    cv::drawMatches(img1, Bow_F->m_keypoints, img2, Cur_F->m_keypoints, ransac_matches, match_image,
//                    cv::Scalar::all(-1),  cv::Scalar::all(-1), std::vector<char>(),
//                            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//    cv::imshow("good matches ", match_image0);
//    cv::imshow("Ransac Matches", match_image);
//    cv::waitKey(0);
    Bow_F->m_orbmatches = ransac_matches;
    return ransac_matches.size();


}

