/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
* Describe	: header of camera class
*/

#ifndef SV3D_CAMERA_H
#define SV3D_CAMERA_H

#include <Eigen/Dense>
#include "../eigen_defs.h"
using namespace Eigen;

namespace sv3d
{
	// 相机的模型通过下式来表达：
	//   P = K[R,t]
	// 其中R和t表示相机相对于世界坐标系的姿态和平移
	// R 为相机坐标系相对于世界坐标系的旋转矩阵
	// t 为相机坐标系相对于世界坐标系的平移矢量
	// 相机坐标系和世界坐标系均为右手系
	class Camera {
	public:
		RMat3	K_;			// 内参矩阵
		RMat3	R_;			// 旋转矩阵
		Vec3	t_;			// 平移矩阵
		RMat34	P_;			// 投影矩阵
		RMat3	KI_;		// 内参矩阵的逆矩阵
		RMat3	RT_;		// 旋转矩阵的逆矩阵

	public:
		inline Camera() = default;
		inline ~Camera() = default;

		/** 通过K、R、t来构造相机，注意这里用了移动语义避免复制操作，传入的参数在执行构造函数后内存销毁 */
		inline Camera(RMat3 K, RMat3 R, Vec3 t) : K_(std::move(K)), R_(std::move(R)), t_(std::move(t)) {
			KI_ = K_.inverse();
			RT_ = R_.transpose();
			P_ << K_ * R_, K_* t_;
		}

		/*****************************************以下是坐标系转换接口*****************************************/
		/** 相机坐标系转换到世界坐标系 */
		inline Vec3 TransformPointC2W(const Vec3& X) const {
			return R_.transpose() * (X - t_);
		}
		/** 世界坐标系到相机坐标系 */
		inline Vec3 TransformPointW2C(const Vec3& X) const {
			return R_ * X + t_;
		}
		/** 相机坐标系到影像坐标系 */
		inline Vec2 TransformPointC2I(const Vec3& X) const {
			auto I = Vec3(K_ * X);
			return Vec2(I[0] / I[2], I[1] / I[2]);
		}
		/** 世界坐标系到影像坐标系 */
		inline Vec2 TransformPointW2I(const Vec3& X) const {
			return TransformPointC2I(TransformPointW2C(X));
		}
		/**  影像坐标系转换到相机坐标系(深度已知：X[2]) */
		inline Vec3 TransformPointI2C(const Vec3& X) const {
			auto Xt = X;
			Xt[0] *= Xt[2]; Xt[1] *= Xt[2];
			return KI_ * Xt;
		}
		/**  影像坐标系转换到世界坐标系(深度已知：X[2]) */
		inline Vec3 TransformPointI2W(const Vec3& X) const {
			return TransformPointC2W(TransformPointI2C(X));
		}
		/*****************************************以上是坐标系转换接口*****************************************/

	};
}

#endif