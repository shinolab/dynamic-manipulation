#include "geometryUtil.hpp"
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

std::vector<Eigen::Vector3f> cornersWorkspaceAll(Eigen::Vector3f const& corner0, Eigen::Vector3f const& corner1) {
	std::vector<Eigen::Vector3f> corners;
	int index[3];
	for (int i = 0; i < 8; i++) { // for the number of the corners
		int res = i;
		for (int j = 0; j < 3; j++) { // for x, y, z
			index[j] = res % 2;
			res /= 2;
		}
		float x = (index[0] == 0 ? corner0.x() : corner1.x());
		float y = (index[1] == 0 ? corner0.y() : corner1.y());
		float z = (index[2] == 0 ? corner0.z() : corner1.z());
		corners.push_back(Eigen::Vector3f(x, y, z));
	}
	return std::move(corners);
}

std::vector<float> range2Points(Eigen::Vector3f const& pos_self, Eigen::Quaternionf const& quo_self, std::vector<Eigen::Vector3f> const& points) {
	std::vector<float> ranges;
	std::for_each(points.begin(), points.end(), [&ranges, &pos_self, &quo_self](Eigen::Vector3f point) {
		ranges.push_back((point - pos_self).dot(quo_self * Eigen::Vector3f::UnitZ()));
		});
	return std::move(ranges);
}

bool isInsideWorkspace(Eigen::Vector3f const& pos, Eigen::Vector3f const& lowerbound, Eigen::Vector3f const& upperbound) {
	Eigen::Vector3f v0 = pos - lowerbound;
	Eigen::Vector3f v1 = pos - upperbound;
	return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
}
