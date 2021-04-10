/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#ifndef SV3D_MATRIX_HOMOGRAPHY_H
#define SV3D_MATRIX_HOMOGRAPHY_H

#include "../eigen_defs.h"


namespace  sv3d
{

	class Homography
	{
	public:
		Homography() = default;
		~Homography() = default;
		
		/**
		 * \brief 单应性矩阵求解
		 * \param p1 视图1上像素点齐次坐标
		 * \param p2 视图2上像素点齐次坐标
		 */
		void Solve(const Mat3X p1, const Mat3X p2);

		/**
		 * \brief 获取单应性矩阵
		 * \return 单应性矩阵
		 */
		Mat3 Value();
	private:
		/**
		 * \brief 四点法求解单应性矩阵
		 * \param p1 视图1上的像素点齐次坐标
		 * \param p2 视图2上的像素点齐次坐标
		 */
		void Solve_FourPoints(const Mat3X p1, const Mat3X p2);

		/* 单应性矩阵数据 */
		Mat3 data_;
	};

}

#endif