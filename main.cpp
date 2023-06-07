#include <iostream>
#include "Eigen/Geometry"
#include "DBoW3/DBoW3.h"
#include "opencv2/opencv.hpp"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Frame.h"
#include "experimental/filesystem"
#include "thread"

std::vector<std::shared_ptr<Frame>> Vec_BOWFrame;

//cv::Mat K = (cv::Mat_<double>(3,3) << 481.2, 0, 319.5, 0, -480.0, 239.5, 0, 0, 1);
cv::Mat K = (cv::Mat_<double>(3,3) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1);

void LoadImages(const std::string &strimagePath, DBoW3::Database &db) {
    std::string pose = strimagePath + "pose.txt";
    std::ifstream fin(pose);
    if (!fin) {
        std::cerr << "cannot find pose file!!!!" << std::endl;
        return;
    }
    std::string line;
    int i = 0;
    while(std::getline(fin, line)) {
        std::string rgb_img = strimagePath + "color/" + std::to_string(i + 1) + ".png";
        std::string depth_img = strimagePath + "depth/" + std::to_string(i + 1) + ".pgm";
        cv::Mat colorImgs = cv::imread(rgb_img);
        cv::Mat depthImgs = cv::imread(depth_img, -1);

        double data[7] = {0};
        for (int k = 0; k < 7; k++) {
            fin >> data[k];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[3]));
        std::cout << "\ni = " << i << "  Pose = \n" << T.matrix() << std::endl;

//        Frame currentFrame = Frame(colorImgs, depthImgs, T, i+1, K);
        std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(colorImgs, depthImgs, T, i+1, K);
        Vec_BOWFrame.push_back(currentFrame);
        db.add(currentFrame->m_des);

        i++;
    }
    std::cout << "database info: " << db << std::endl;
}



int main(int argc, char** argv) {
    DBoW3::Vocabulary vocab(argv[1]);
    if (vocab.empty()) {
        std::cerr << "Vocabulary does not exist." << std::endl;
        return -1;
    }

    DBoW3::Database db(vocab, false, 0);
    std::string img_path = argv[2];
    LoadImages(img_path, db);

    auto t1 = clock();
    cv::Mat im = cv::imread(argv[3]);
    DBoW3::QueryResults ret;
    std::shared_ptr<Frame> cur = std::make_shared<Frame>(im, -1, K);

    db.query(cur->m_des, ret, 5);
    std::cout << "searching for image "  << cur->FrameId <<  " returns "  << ret << std::endl << std::endl;
    std::cout << "ret = " << ret.at(1).Id + 1 << std::endl;



    // 使用多线程实现ORB特征匹配
//    int num_theards = ret.size();
//    std::cout << "theards = " << num_theards << std::endl;
//    std::vector<std::thread> threads(num_theards);
//    std::mutex matches_mutex, matches_mutex2;
//    ORBmatcher matcher(0.9, true);
//    std::vector<std::pair<int, int>> vec_match;
//    vec_match.reserve(num_theards);
//    for (int i = 0; i < num_theards; i++) {
//        threads[i] = std::thread([&,i] {
//            std::shared_ptr<Frame> cur_copy;
//            std::lock_guard<std::mutex> lock(matches_mutex);
//            {
////                Frame cur_copy = Frame(cur);
//                cur_copy = std::make_shared<Frame>(cur);
//            }
//            int matches = matcher.SearchForReplace(img_path, Vec_BOWFrame[ret.at(i).Id], cur_copy);
//            std::cout << "matches = " << matches << std::endl;
//            std::lock_guard<std::mutex> lock1(matches_mutex2);
//            {
//                vec_match.push_back(std::make_pair(matches,ret.at(i).Id));
//            }
//        });
//    }
//    for (auto &thread : threads) {
//        thread.join();
//    }



    ORBmatcher matcher(0.9, true);
    std::vector<std::pair<int, int>> vec_match;
    for (int i = 0; i < ret.size(); i++) {
        int matches = matcher.SearchForReplace(img_path, Vec_BOWFrame[ret.at(i).Id], cur);
        std::cout << "matches = " << matches << std::endl;
        vec_match.push_back(std::make_pair(matches,ret.at(i).Id));
    }
    std::sort(vec_match.begin(), vec_match.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
        return a.first > b.first;
    });

    for (auto &m : vec_match) {
        std::cout << "matches = " << m.first << " i = " << m.second << std::endl;
    }

    cur->GetMapPoints(Vec_BOWFrame[vec_match[1].second]);

    // by OpenCV
    cv::Mat r, t, R;
    cv::solvePnP(Vec_BOWFrame[vec_match[1].second]->m_pts3d, cur->m_pts2d, K, cv::Mat() ,r, t, false);
    cv::Rodrigues(r,R);
    std::cout << "R = \n" << R << std::endl;
    std::cout << "t = \n" << t << std::endl;

    int inilers = Optimizer::PoseOptimization(Vec_BOWFrame[vec_match[1].second], cur);
    std::cout << "BA: inilers = " <<  inilers << std::endl;

    std::cout << "Bow_F pose = " << Vec_BOWFrame[vec_match[1].second]->m_pose.matrix() << std::endl;
    double mu = 180 / 3.14;
    cv::Mat Tcw = cur->m_Tcr * Converter::toCvMat(Vec_BOWFrame[vec_match[1].second]->m_pose.inverse().matrix());
    std::cout << "Tcw = \n" << Tcw << std::endl;
    std::cout << "pitch = " << asin(-Tcw.at<double>(2,0)) * mu << " roll = " << atan2(Tcw.at<double>(2,1),Tcw.at<double>(2,2)) * mu
              << " yaw = " << atan2(Tcw.at<double>(1,0),Tcw.at<double>(0,0)) * mu << std::endl;
//
    std::cout << "Eigen T_tcw = \n" << Vec_BOWFrame[vec_match[0].second]->m_pose.inverse().matrix() << std::endl;
    cv::Mat t_Tcw = Converter::toCvMat(Vec_BOWFrame[vec_match[0].second]->m_pose.inverse().matrix());
    std::cout << "True Tcw = \n" << t_Tcw << std::endl;
    std::cout << "t_pitch = " << asin(-t_Tcw.at<double>(2,0)) * mu << " t_roll = " << atan2(t_Tcw.at<double>(2,1),t_Tcw.at<double>(2,2)) * mu
              << " t_yaw = " << atan2(t_Tcw.at<double>(1,0),t_Tcw.at<double>(0,0)) * mu << std::endl;

    auto t2 = clock();
    std::cout << "time = " << double(t2 - t1) / CLOCKS_PER_SEC << std::endl;
    return 0;

}
