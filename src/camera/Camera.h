/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
* Describe	: header of camera class
*/

#pragma once
#include <Eigen/Dense>

using namespace Eigen;

// 相机的模型通过下式来表达：
//   P = K[R,t]
// 其中R和t表示相机相对于世界坐标系的姿态和平移
// R 为相机坐标系相对于世界坐标系的旋转矩阵
// t 为相机坐标系相对于世界坐标系的平移矢量
// 相机坐标系和世界坐标系均为右手系
typedef Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dr;
typedef Matrix<double, 3, 4, Eigen::RowMajor> Matrix34dr;
class Camera {
private:
	Matrix3dr	K_;			// 内参矩阵
    Matrix3dr	R_;			// 旋转矩阵
	Vector3d	t_;			// 平移矩阵
	Matrix34dr	P_;			// 投影矩阵
	Matrix3dr	KI_;		// 内参矩阵的逆矩阵
	Matrix3dr	RT_;		// 旋转矩阵的逆矩阵
	
public:
    inline Camera() = default;
	inline ~Camera() = default;
	
	/** 通过K、R、t来构造相机，注意这里用了移动语义避免复制操作，传入的参数在执行构造函数后内存销毁，建议使用临时变量构造 */
	inline Camera(Matrix3dr K, Matrix3dr R, Vector3d t) : K_(std::move(K)), R_(std::move(R)), t_(std::move(t)) {
		KI_ = K_.inverse();
		RT_ = R_.transpose();
		P_ << K_*R_, K_*t_;
	}
   
	/*****************************************以下是坐标系转换接口*****************************************/
	/** 相机坐标系转换到世界坐标系 */
	inline Vector3d TransformPointC2W(const Vector3d& X) const {
		return R_.transpose() * (X - t_);
	}
	/** 世界坐标系到相机坐标系 */
	inline Vector3d TransformPointW2C(const Vector3d& X) const {
		return R_*X + t_;
	}
	/** 相机坐标系到影像坐标系 */
	inline Vector2d TransformPointC2I(const Vector3d& X) const {
		auto I = Vector3d(K_*X);
		return Vector2d(I(0) / I(2), I(1) / I(2));
	}
	/** 世界坐标系到影像坐标系 */
	inline Vector2d TransformPointW2I(const Vector3d& X) const {
		return TransformPointC2I(TransformPointW2C(X));
	}
	/**  影像坐标系转换到相机坐标系(深度已知：X[2]) */
	inline Vector3d TransformPointI2C(const Vector3d& X) const {
		auto Xt = X;
		Xt[0] *= Xt[2]; Xt[1] *= Xt[2];
		return KI_*Xt;
	}
	/**  影像坐标系转换到世界坐标系(深度已知：X[2]) */
	inline Vector3d TransformPointI2W(const Vector3d& X) const {
		return TransformPointC2W(TransformPointI2C(X));
	}
	/*****************************************以上是坐标系转换接口*****************************************/
	
};