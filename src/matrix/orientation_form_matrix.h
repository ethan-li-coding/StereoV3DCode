#ifndef SV3D_RELATIVE_ORIENTATION_MATRIX_H
#define SV3D_RELATIVE_ORIENTATION_MATRIX_H

#include "../eigen_defs.h"

namespace sv3d {
	
	/**
	 * \brief 从本质矩阵恢复像对的相对R/t，返回相机前方有最多点的解
	 * \param p1[in]		同名点对在左视图的点集 
	 * \param p2[in]		同名点对在右视图的点集
	 * \param k1_mat[in]	左视图内参矩阵
	 * \param k2_mat[in]	右视图内参矩阵
	 * \param E[in]			本质矩阵
	 * \param R[out]		R矩阵
	 * \param t[out]		t矢量 
	 * \param uniqueness_ratio[in]	唯一性比例（次最佳解和最佳解相机前的点数比例阈值，小于阈值则返回false）
	 * \return true:求解成功 false:求解失败
	 */
	bool OrientationFormEssential(const Mat3X & p1, const Mat3X & p2,
		const RMat3& k1_mat, const RMat3& k2_mat,
		const Mat3& E,
		Mat3& R, Vec3& t,
		const double uniqueness_ratio = 0.7);
}

#endif