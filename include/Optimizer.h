//
// Created by bing on 2023/6/5.
//

#ifndef REPLACE_OPTIMIZER_H
#define REPLACE_OPTIMIZER_H

#include "Frame.h"
#include "../Thirdparty/g2o/g2o/core/block_solver.h"
#include "../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "../Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "../Thirdparty/g2o/g2o/core/block_solver.h"
#include "../Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include <eigen3/Eigen/Core>

#include "Converter.h"
class Frame;
class Converter;
class Optimizer {
public:
    Optimizer() {}

    static int PoseOptimization(std::shared_ptr<Frame> Bow_F, std::shared_ptr<Frame> Cur_F);

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> BlockSolver_6_3;
    typedef g2o::LinearSolverDense<BlockSolver_6_3::PoseMatrixType> LinearSolverType;
};

#endif //REPLACE_OPTIMIZER_H
