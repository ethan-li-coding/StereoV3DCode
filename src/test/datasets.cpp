/* -*-c++-*- StereoV3D - Copyright (C) 2021.
* Author	: Ethan Li<ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/StereoV3DCode
*/

#include "datasets.h"
#include <random>

void sv3d::SimulativeStereoDataset::GenarateHomonymyPairs(const unsigned& k, Mat3X& p1, Mat3X& p2)
{
	// 在立体相机的空间内随机生成k个空间点
	const double kDepthMin = 300., kDepthMax = 500.;

	// 在最小深度平面计算四个角点的世界坐标（世界坐标系=左相机坐标系）
	Vec3 x1(0, 0, kDepthMin), x2(0, h-1, kDepthMin),
		x3(w-1, 0, kDepthMin), x4(w - 1, h - 1, kDepthMin);
	auto P1 = cam1.TransformPointI2W(x1);
	auto P2 = cam1.TransformPointI2W(x2);
	auto P3 = cam1.TransformPointI2W(x3);
	auto P4 = cam1.TransformPointI2W(x4);

	// 生成一个空间立方体，在立方体范围内随机生成空间点
	const auto z_min(kDepthMin), z_max(kDepthMax);
	const auto x_min = (std::min)(P1.data()[0], P2.data()[0]);
	const auto x_max = (std::max)(P3.data()[0], P4.data()[0]);
	const auto y_min = (std::min)(P1.data()[0], P3.data()[0]);
	const auto y_max = (std::max)(P2.data()[0], P4.data()[0]);

	// 随机生成k个空间点
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> rand(0.0f, 1.0f);

	const auto x_range = x_max - x_min;
	const auto y_range = y_max - y_min;
	const auto z_range = z_max - z_min;
	p1.resize(3, k);
	p2.resize(3, k);
	for (unsigned n = 0; n < k; n++) {
		Vec3 w;
		w[0] = rand(gen) * x_range + x_min;
		w[1] = rand(gen) * y_range + y_min;
		w[2] = rand(gen) * z_range + z_min;

		Vec2 x1 = cam1.TransformPointW2I(w);
		Vec2 x2 = cam2.TransformPointW2I(w);

		p1.data()[3 * n] = x1[0]; p1.data()[3 * n + 1] = x1[1]; p1.data()[3 * n + 2] = 1.;
		p2.data()[3 * n] = x2[0]; p2.data()[3 * n + 1] = x2[1]; p2.data()[3 * n + 2] = 1.;
	}
}

void sv3d::SimulativeStereoDataset::GenarateHomonymyPairsInPlane(const unsigned& k, Mat3X& p1, Mat3X& p2, Mat3& H)
{
	// 在立体相机的某深度平面内随机生成k个空间点
	const double kDepth = 400.;

	// 在深度平面计算四个角点的世界坐标（世界坐标系=左相机坐标系）
	Vec3 x1(0, 0, kDepth), x2(0, h - 1, kDepth),
		x3(w - 1, 0, kDepth), x4(w - 1, h - 1, kDepth);
	auto P1 = cam1.TransformPointI2W(x1);
	auto P2 = cam1.TransformPointI2W(x2);
	auto P3 = cam1.TransformPointI2W(x3);
	auto P4 = cam1.TransformPointI2W(x4);

	// 生成一个空间截面，截面范围内随机生成空间点
	const auto z(kDepth);
	const auto x_min = (std::min)(P1.data()[0], P2.data()[0]);
	const auto x_max = (std::max)(P3.data()[0], P4.data()[0]);
	const auto y_min = (std::min)(P1.data()[0], P3.data()[0]);
	const auto y_max = (std::max)(P2.data()[0], P4.data()[0]);

	// 随机生成k个空间点
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> rand(0.0f, 1.0f);

	const auto x_range = x_max - x_min;
	const auto y_range = y_max - y_min;
	p1.resize(3, k);
	p2.resize(3, k);
	for (unsigned n = 0; n < k; n++) {
		Vec3 w;
		w[0] = rand(gen) * x_range + x_min;
		w[1] = rand(gen) * y_range + y_min;
		w[2] = z;

		Vec2 x1 = cam1.TransformPointW2I(w);
		Vec2 x2 = cam2.TransformPointW2I(w);

		p1.data()[3 * n] = x1[0]; p1.data()[3 * n + 1] = x1[1]; p1.data()[3 * n + 2] = 1.;
		p2.data()[3 * n] = x2[0]; p2.data()[3 * n + 1] = x2[1]; p2.data()[3 * n + 2] = 1.;
	}

	// 计算单应性矩阵H, p2 = H*p1
	Mat3X t(3, 1); t << cam2.t_[0], cam2.t_[1], cam2.t_[2];
	MatXX nt(1, 3); nt << 0, 0, 1. / z;
	H = cam2.K_ * (cam2.R_ + t * nt) * cam1.K_.inverse();
}
