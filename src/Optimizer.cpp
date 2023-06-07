//
// Created by bing on 2023/6/5.
//

#include "Optimizer.h"
#include "Converter.h"

int Optimizer::PoseOptimization(std::shared_ptr<Frame> Bow_F, std::shared_ptr<Frame> Cur_F) {
    g2o::SparseOptimizer optimizer;

    auto linearSolver = new g2o::LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
    BlockSolver_6_3 *solver_ptr = new BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
//    vSE3->setEstimate(g2o::SE3Quat(Bow_F->m_pose.rotation(), Bow_F->m_pose.translation()));
//    Eigen::Matrix<double, 3, 3>;
    Eigen::Matrix<double, 3, 1> t(0,0,0);
    vSE3->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), t));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    const int N = Bow_F->m_pts3d.size();
    std::cout << "N = " << N << std::endl;

    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdge;
    vpEdge.reserve(N);

    for (size_t i = 0; i < N; i++) {
        cv::Point3d p3d = Bow_F->m_pts3d[i];
        cv::Point2d p2d = Cur_F->m_pts2d[i];
        Eigen::Matrix<double, 2, 1> obs;
        obs << p2d.x, p2d.y;
//        std::cout << "obs = " << obs.transpose() << std::endl;
        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

        e->fx = Bow_F->m_K.at<double>(0,0);
        e->fy = Bow_F->m_K.at<double>(1,1);
        e->cx = Bow_F->m_K.at<double>(0,2);
        e->cy = Bow_F->m_K.at<double>(1,2);

        e->Xw << p3d.x, p3d.y, p3d.z;
//        std::cout << "Xw = " << e->Xw.transpose() << std::endl;

        vpEdge.push_back(e);
        optimizer.addEdge(e);
    }

    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const int its[4]={10,10,10,10};

    int nBad = 0;
    std::vector<bool> isBad;
    isBad.reserve(N);
    for (size_t it = 0; it < 4; it++) {
//        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        nBad = 0;
        isBad.clear();
        for (size_t i = 0; i < vpEdge.size(); i++) {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdge[i];
            e->computeError();
            float ch = e->chi2();
            if (ch > chi2Mono[it]) {
                e->setLevel(1);
                isBad.push_back(true);
                nBad++;
            } else {
                isBad.push_back(false);
                e->setLevel(0);
            }
//            std::cout << "ch = " << ch << std::endl;
        }
    }

    g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    std::cout << "Tcw = \n" << SE3quat_recov << std::endl;

    Cur_F->m_Tcr = Converter::toCvMat(SE3quat_recov);

//    std::cout << "Tcw = \n" << SE3quat_recov * Bow_F->m_pose.inverse().matrix() << std::endl;
//    std::cout << "Twc = \n" << (SE3quat_recov * Bow_F->m_pose.inverse().matrix()).inverse() << std::endl;
//    std::cout << "isBad size = " << isBad.size() << std::endl;

    // 计算重投影误差
//    double error = 0.0;
//    for (int k = 0; k < N; k++) {
//        if (isBad[k])
//            continue;
//        cv::Point3d p3d = Bow_F->m_pts3d[k];
//        cv::Point2d p2d = Cur_F->m_pts2d[k];
//
//        Eigen::Vector3d p3d_c1,p3d_c2;
//        p3d_c1 << p3d.x, p3d.y, p3d.z;
//        p3d_c2 = SE3quat_recov.rotation() * p3d_c1 + SE3quat_recov.translation();
//        double u = Bow_F->m_K.at<double>(0,0) * p3d_c2(0) / p3d_c2(2) + Bow_F->m_K.at<double>(0,2);
//        double v = Bow_F->m_K.at<double>(1,1) * p3d_c2(1) / p3d_c2(2) + Bow_F->m_K.at<double>(1,2);
//
//        double err = sqrt((u - p2d.x) * (u - p2d.x) + (v - p2d.y) * (v - p2d.y));
//        std::cout << "err = " << err << std::endl;
//        error += sqrt((u - p2d.x) * (u - p2d.x) + (v - p2d.y) * (v - p2d.y));
//
//    }
//    std::cout << "error = " << error << std::endl;

    return N -  nBad;

}

