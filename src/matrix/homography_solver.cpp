/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#include "homography_solver.h"

void sv3d::HomographySolver::Solve(const Mat3X p1, const Mat3X p2)
{
	Solve_FourPoints(p1, p2);
}

sv3d::Mat3 sv3d::HomographySolver::Value()
{
	return data_;
}

void sv3d::HomographySolver::Solve_FourPoints(const Mat3X p1, const Mat3X p2)
{
	assert(p1.cols() >= 4);
	assert(p1.rows() == p2.rows());
	assert(p1.cols() == p2.cols());

	// 构建线性方程组Ah=b的系数矩阵A和矩阵b
	auto np = p1.cols();
	RMatXX a_mat(2 * np, 8), at(8, 2 * np), ata(8, 8);
	MatXX b_mat(2 * np, 1), atb(8, 1);
	for (int n = 0; n < np; n++) {
		const auto p1_x = p1.data()[3 * n], p1_y = p1.data()[3 * n + 1];
		const auto p2_x = p2.data()[3 * n], p2_y = p2.data()[3 * n + 1];
		auto dat = a_mat.data() + 8 * 2 * n;
		dat[0] = p1_x; dat[1] = p1_y; dat[2] = 1; dat[3] = dat[4] = dat[5] = 0;
		dat[6] = -p1_x * p2_x; dat[7] = -p1_y * p2_x;
		dat = a_mat.data() + 8 * (2 * n + 1);
		dat[0] = dat[1] = dat[2] = 0; dat[3] = p1_x; dat[4] = p1_y; dat[5] = 1;
		dat[6] = -p1_x * p2_y; dat[7] = -p1_y * p2_y;
		b_mat.data()[2 * n] = p2_x; b_mat.data()[2 * n + 1] = p2_y;
	}

	// 解Ah=b
	at = a_mat.transpose();
	ata = at * a_mat;
	atb = at * b_mat;
	MatXX h(8, 1);
	h = ata.inverse() * atb;

	// 构造单应性矩阵H
	data_ = Eigen::Map<const RMat3>(h.data());
	data_.data()[8] = 1.;
}
