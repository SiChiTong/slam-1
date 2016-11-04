#include "slam/optimization/eight_point.hpp"


namespace slam {
namespace optimization {

MatX kronecker_product(MatX A, MatX B)
{
    MatX C;
    double a;

    // setup
    C.resize((A.rows() * B.rows()), (A.cols() * B.cols()));

    // calculate kronecker product
    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            a = A(i, j);
            C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = a * B;
        }
    }

    return C;
}

EightPoint::EightPoint(void)
{
    this->configured = false;

    this->image_width = 0;
    this->image_height = 0;
    this->N << 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0;
}

int EightPoint::configure(int image_width, int image_height)
{
    this->configured = true;

    this->image_width = image_width;
    this->image_height = image_height;
    this->N << 2.0 / image_width, 0.0, -1.0,
               0.0, 2.0 / image_height, -1.0,
               0.0, 0.0, 1.0;

    return 0;
}

void EightPoint::normalizePoints(MatX &pts1, MatX &pts2)
{
    pts1 = (this->N * pts1.transpose()).transpose();
    pts2 = (this->N * pts2.transpose()).transpose();
}

void EightPoint::formMatrixA(MatX &pts1, MatX &pts2, MatX &A)
{
    int rows;
    VecX x1, x2, y1, y2, ones;

    // setup
    rows = pts1.rows();
    x1 = pts1.block(0, 0, rows, 1);
    x2 = pts2.block(0, 0, rows, 1);
    y1 = pts1.block(0, 1, rows, 1);
    y2 = pts2.block(0, 1, rows, 1);
    ones = MatX::Ones(rows, 1);

    // form matrix A; Af = 0
    A.resize(rows, 9);
    A.block(0, 0, rows, 1) = x1.cwiseProduct(x2);
    A.block(0, 1, rows, 1) = y1.cwiseProduct(x2);
    A.block(0, 2, rows, 1) = x2;
    A.block(0, 3, rows, 1) = x1.cwiseProduct(y2);
    A.block(0, 4, rows, 1) = y1.cwiseProduct(y2);
    A.block(0, 5, rows, 1) = y2;
    A.block(0, 6, rows, 1) = x1;
    A.block(0, 7, rows, 1) = y1;
    A.block(0, 8, rows, 1) = ones;
}

void EightPoint::approximateFundamentalMatrix(MatX &A, MatX &F)
{
    MatX U, V;
    Eigen::JacobiSVD<MatX> svd;

    // computing SVD of A
    svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();

    // form approximate matrix F by
    // extracting 9th-col of V (9 x 1) to form F (3 x 3)
    F = V.block(0, 8, 9, 1);
    F.resize(3, 3);
    F.transposeInPlace();
}

void EightPoint::refineFundamentalMatrix(MatX &F)
{
    Mat3 D;
    MatX U, V, S;
    Eigen::JacobiSVD<MatX> svd;

    // recompute SVD using approximate matrix F
    svd.compute(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    D = MatX::Zero(3, 3);
    D(0, 0) = S(0);
    D(1, 1) = S(1);
    D(2, 2) = 0;
    F = U * D * V.transpose();
}

void EightPoint::denormalizeFundamentalMatrix(MatX &F)
{
    F = this->N.transpose() * F * this->N;
}

int EightPoint::estimate(MatX &pts1, MatX &pts2)
{
    VecX S;
    MatX A, F;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts1.size() != pts2.size()) {
        return -2;
    }

    // calculate fundamental matrix
    this->normalizePoints(pts1, pts2);
    this->formMatrixA(pts1, pts2, A);
    this->approximateFundamentalMatrix(A, F);
    this->refineFundamentalMatrix(F);
    this->denormalizeFundamentalMatrix(F);

    std::cout << F << std::endl;

    return 0;
}

}  // end of optimization namespace
}  // end of slam namespace
