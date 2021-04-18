#ifndef SV3D_MATRIX_UTILS_H
#define SV3D_MATRIX_UTILS_H

#include "../eigen_defs.h"
#include <vector>

namespace sv3d
{
	/**
	 * \brief 从本质矩阵解算R,t
	 * \param[in] E		本质矩阵
	 * \param[out] R_vec R矩阵的可能值集合
	 * \param[out] t_vec t矩阵的可能值集合
	 */
	void SolveRtFromEssential(const Mat3& E, std::vector<Mat3>& R_vec, std::vector<Vec3>& t_vec);

	/**
	 * \brief 双目前方交会求解世界坐标系下的空间点坐标
	 * \param[in] p1 同名点在左视图的像素坐标
	 * \param[in] K1 左视图内参矩阵
	 * \param[in] R1 左视图外参R矩阵
	 * \param[in] t1 左视图外参t矩阵
	 * \param[in] p2 同名点在右视图的像素坐标
	 * \param[in] K2 右视图内参矩阵
	 * \param[in] R2 右视图外参R矩阵
	 * \param[in] t2 右视图外参t矩阵
	 * \param[out] X 空间点坐标
	 * return true: 空间点在双相机前方 
	 */
	bool Triangulate2View(const Vec3& p1, const RMat3& K1, const Mat3& R1, const Vec3& t1,
						  const Vec3& p2, const RMat3& K2, const Mat3& R2, const Vec3& t2,
						  Vec3& X);
}

#endif