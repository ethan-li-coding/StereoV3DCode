/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#include "essential_solver.h"

void sv3d::EssentialSolver::Solve(const Mat3X p1, const Mat3X p2, const RMat3 k1_mat, const RMat3 k2_mat, const SOLVE_TYPE& solver_type)
{
	assert(p1.cols() >= 8);
	assert(p1.rows() == p2.rows());
	assert(p1.cols() == p2.cols());
	
	// 通过内参矩阵k将p转换到x，x = k_inv*p
	Mat3X x1(3,p1.cols()), x2(3,p2.cols());

	const RMat3 k1i = k1_mat.inverse();
	const RMat3 k2i = k2_mat.inverse();
	x1 = k1i * p1;
	x2 = k2i * p2;

	// 求解
	Solve(x1, x2, solver_type);
}

void sv3d::EssentialSolver::Solve(const Mat3X x1, const Mat3X x2, const SOLVE_TYPE& solver_type)
{
	switch (solver_type) {
	case EIGHT_POINTS:
		Solve_EightPoints(x1, x2);
	default:
		break;
	}
}

sv3d::Mat3 sv3d::EssentialSolver::Value()
{
	return data_;
}

void sv3d::EssentialSolver::Solve_EightPoints(const Mat3X x1, const Mat3X x2)
{
	assert(x1.cols() >= 8);
	assert(x1.rows() == x2.rows());
	assert(x1.cols() == x2.cols());

	// 构建线性方程组的系数矩阵A
	auto np = x1.cols();
	RMatX9 a_mat(np, 9);
	for (int n = 0; n < np; n++) {
		const auto x1_x = x1.data()[3 * n];
		const auto x1_y = x1.data()[3 * n + 1];
		const auto x2_x = x2.data()[3 * n];
		const auto x2_y = x2.data()[3 * n + 1];
		const auto dat = a_mat.data() + 9 * n;
		dat[0] = x1_x * x2_x; dat[1] = x2_x * x1_y; dat[2] = x2_x;
		dat[3] = x1_x * x2_y; dat[4] = x1_y * x2_y; dat[5] = x2_y;
		dat[6] = x1_x; dat[7] = x1_y; dat[8] = 1;
	}
	
	// 求解系数矩阵的最小特征值，对应的特征向量即矢量 e
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9, 9>> solver(a_mat.transpose()*a_mat);
	const Vec9 e = solver.eigenvectors().leftCols<1>();
	
	// 矢量 e 构造本质矩阵 E
	data_ = Eigen::Map<const RMat3>(e.data());
	
	// 调整 E 矩阵使满足内在性质：奇异值为[σ σ 0]
	Eigen::JacobiSVD<Mat3> usv(data_, Eigen::ComputeFullU | Eigen::ComputeFullV);
	auto s = usv.singularValues();
	const auto a = s[0];
	const auto b = s[1];
	s << (a + b) / 2.0, (a + b) / 2.0, 0.0;
	data_ = usv.matrixU() * s.asDiagonal() * usv.matrixV().transpose();
}
