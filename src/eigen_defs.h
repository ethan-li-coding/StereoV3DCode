/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#ifndef SV3D_EIGEN_DEFS_H
#define SV3D_EIGEN_DEFS_H

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

namespace sv3d
{

	// 3d 矢量 (double类型)
	using Vec3 = Eigen::Vector3d;

	// 3d 矢量 (float类型)
	using Vec3f = Eigen::Vector3f;
		
	// 3d 矢量 (double类型)
	using Vec2 = Eigen::Vector2d;
		
	/// 2d 矢量 (float类型)
	using Vec2f = Eigen::Vector2f;

	// 9d 矢量
	using Vec9 = Eigen::Matrix<double, 9, 1>;
	
	// 3x3 矩阵 (double类型)
	using Mat3 = Eigen::Matrix<double, 3, 3>;
		
	// 3x4 矩阵 (double类型)
	using Mat34 = Eigen::Matrix<double, 3, 4>;
		
	// 3x3 矩阵 (double类型)（行主序）
	using RMat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

	// 3x4 矩阵 (double类型)
	using RMat34 = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>;
		
	// 2xN 矩阵 (double类型)
	using Mat2X = Eigen::Matrix<double, 2, Eigen::Dynamic>;
		
	// 3xN 矩阵 (double类型)
	using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

	// Nx9 矩阵 (double类型)
	using MatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9>;

	// Nx9 矩阵 (double类型)（行主序）
	using RMatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9, Eigen::RowMajor>;

	// NxM 矩阵 (double类型)
	using MatXX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

	// NxM 矩阵 (double类型)（行主序）
	using RMatXX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
}

#endif