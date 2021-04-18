#include "orientation_form_matrix.h"

#include "matrix_utils.h"
using std::vector;

bool sv3d::OrientationFormEssential(const Mat3X& p1, const Mat3X& p2,
											const RMat3& k1_mat, const RMat3& k2_mat,
											const Mat3& E, Mat3& R, Vec3& t,
                                            const double uniqueness_ratio)
{
	assert(p1.rows() == p2.rows());
	assert(p1.cols() == p2.cols());

	// 分解E矩阵得到R,t候选值
	std::vector<Mat3> R_vec;
	std::vector<Vec3> t_vec;
	SolveRtFromEssential(E,R_vec,t_vec);
	if(R_vec.size()!=2u || t_vec.size()!=2) {
		return false;
	}
	
	// R,t各两组值，共4中组合
	// 计算各组合在相机前方的观测点数，点数最多的组合为最优解
	struct Pose {
		Mat3* R;
		Vec3* t;
		Pose(Mat3& R_,Vec3& t_):R(&R_),t(&t_) {}
	};
	vector<Pose> pose_vec;
	pose_vec.emplace_back(R_vec[0], t_vec[0]); pose_vec.emplace_back(R_vec[0], t_vec[1]);
	pose_vec.emplace_back(R_vec[1], t_vec[0]); pose_vec.emplace_back(R_vec[1], t_vec[1]);

	
	Mat3 R0; R0.setIdentity();
	Vec3 t0; t0.setZero();

	vector<unsigned> num_front_points(4);		// 保存在双相机前方的点数

	// 选择双相机前方点数最多的Pose作为最优的Pose
	unsigned max_front_points = 0;
	int max_idx = -1;
	for (int k = 0; k < 4; k++) {
		auto& pose = pose_vec[k];
		const auto num_pts = p1.cols();
		for(unsigned n = 0u; n < num_pts;n++) {
			const Vec3& x1 = p1.col(n);
			const Vec3& x2 = p2.col(n);
			Vec3 X;
			// 三角化计算空间点坐标，并判断空间点坐标是否在两个相机前方
			if(Triangulate2View(x1, k1_mat, R0, t0, x2, k2_mat, *pose.R, *pose.t, X)) {
				++num_front_points[k];
			}
		}
		if(max_front_points < num_front_points[k]) {
			max_front_points = num_front_points[k];
			max_idx = k;
		}
	}

	if (max_idx == -1)
		return false;

	// 输出最优的R,t
	R = *pose_vec[max_idx].R;
	t = *pose_vec[max_idx].t;

	// 计算唯一性
	std::sort(num_front_points.begin(), num_front_points.end());

	const double ratio = num_front_points.rbegin()[1] / static_cast<double>(num_front_points.rbegin()[0]);
	return ratio < uniqueness_ratio;
}
