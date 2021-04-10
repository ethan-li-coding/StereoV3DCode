/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#ifndef SV3D_TEST_DATASETS_H
#define SV3D_TEST_DATASETS_H
#include "../eigen_defs.h"
#include "Camera.h"

namespace sv3d
{
	struct SimulativeStereoDataset
	{
		unsigned w, h;		// 影像宽高
		Camera	cam1, cam2;	// 两个相机

		Mat3 E,F;
		
		SimulativeStereoDataset() {
			
			// 模拟构造一组双目系统
			w = 640; h = 480;
			RMat3 K1,K2,R,RI;
			Vec3 t0, t;
			K1 << 5.3398796382502758e+02, 0., 3.2838647583744523e+02,
				  0.,5.2871083166023755e+02, 2.3684273091753437e+02,
				  0., 0., 1.;
			K2 << 5.3398796382502758e+02, 0., 3.1377033111753229e+02,
				  0., 5.2871083166023755e+02, 2.4187045690014719e+02,
				  0., 0., 1.;
			R << 9.9997149579080535e-01, 4.8210116286829859e-03, 5.8108048302649620e-03,
				-4.8866671827332113e-03, 9.9992378062933918e-01, 1.1338139872769284e-02,
				-5.7557006302035411e-03, -1.1366212157327780e-02, 9.9991883727203090e-01;
			t << -3.3427086946183784e+00, 4.6825941478185688e-02, 3.6523737018390699e-03;

			RI.setIdentity();
			t0.setZero();
			cam1 = Camera(K1, RI, t0);
			cam2 = Camera(K2, R, t);

			// 求解本质矩阵
			Mat3 tx;
			tx << 0., -cam2.t_[2], cam2.t_[1],
				cam2.t_[2], 0., -cam2.t_[0],
				-cam2.t_[1], cam2.t_[0], 0;
			E = tx * R;
			
			// 求解基础矩阵
			F = cam2.K_.inverse().transpose() * E * cam1.K_.inverse();
		}

		/**
		 * \brief 生成同名点对
		 * \param k		要生成的同名点对数量
		 * \param p1	相机1上的像素点
		 * \param p2	相机2上的像素点
		 */
		void GenarateHomonymyPairs(const unsigned& k, Mat3X& p1,Mat3X& p2);

		
		/**
		 * \brief 生成一个平面内的同名点对
		 * \param k		要生成的同名点对数量
		 * \param p1	相机1上的像素点
		 * \param p2	相机2上的像素点
		 * \param H		p1,p2的单应性变换矩阵，p2 = H*p1
		 */
		void GenarateHomonymyPairsInPlane(const unsigned& k, Mat3X& p1, Mat3X& p2, Mat3& H);
	};
}

#endif