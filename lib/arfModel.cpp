#ifndef _ARF_MODEL_
#define _ARF_MODEL_

#include "arfModel.hpp"
#include <Eigen\Geometry>
#include <unsupported/Eigen/Splines>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

/*
returns -1 if an arg is smaller than any elem of range
returns range.size() if an arg is larger than any elem of range
*/
int SearchIntervalIndex(float arg, const Eigen::VectorXf& range) {
	return static_cast<int>((Eigen::VectorXf(range.size()).setConstant(arg) - range).cwiseSign().cwiseMax(0).sum());
}

float linearInterp2d(
	float argRow,
	float argCol,
	const Eigen::VectorXf& argDataPointsRow,
	const Eigen::VectorXf& argDataPointsCol,
	const Eigen::MatrixXf& valueDataPoints
) {
	auto indexIntervalArgRow = std::max(0, std::min(SearchIntervalIndex(argRow, argDataPointsRow), static_cast<int>(argDataPointsRow.size()) - 2));
	auto indexIntervalArgCol = std::max(0, std::min(SearchIntervalIndex(argCol, argDataPointsCol), static_cast<int>(argDataPointsCol.size()) - 2));
	auto indexIntervalArgRowNext = indexIntervalArgRow + 1;
	auto indexIntervalArgColNext = indexIntervalArgCol + 1;
	float f00 = valueDataPoints(indexIntervalArgRow, indexIntervalArgCol);
	float f10 = valueDataPoints(indexIntervalArgRow, indexIntervalArgColNext);
	float f01 = valueDataPoints(indexIntervalArgRowNext, indexIntervalArgCol);
	float f11 = valueDataPoints(indexIntervalArgRowNext, indexIntervalArgColNext);
	float r0 = argDataPointsRow[indexIntervalArgRow];
	float r1 = argDataPointsRow[indexIntervalArgRowNext];
	float c0 = argDataPointsCol[indexIntervalArgCol];
	float c1 = argDataPointsCol[indexIntervalArgColNext];
	float dr = (argRow - r0) / (r1 - r0);
	float dc = (argCol - c0) / (c1 - c0);
	return f00 * (1 - dr) * (1 - dc) + f10 * dr * (1 - dc) + f01 * (1 - dr) * dc + f11 * dr * dc;
}

arfModelLinearBase::~arfModelLinearBase() {}

arfModelConstant::arfModelConstant(float intensity) :m_intensity(intensity) {}

Eigen::MatrixXf arfModelConstant::arf(const Eigen::MatrixXf &posRel, const Eigen::MatrixXf &eulerAnglesAUTD)
{
	return m_intensity * posRel.colwise().normalized();
}

Eigen::MatrixXf arfModelConstant::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) {
	return arf(posRel, Eigen::MatrixXf());
}

Eigen::MatrixXf arfModelExperimentalPoly::arf(const Eigen::MatrixXf& _posRel, const Eigen::MatrixXf& eulerAnglesAUTD)
{
	Eigen::MatrixXf posRel = _posRel / 1000;
	Eigen::RowVector4f arfCoefficient;
	arfCoefficient << 0.3683, 0.6913, -0.9431, 0.2769; arfCoefficient *= 9.8; // [mN]
	Eigen::RowVectorXf dist = posRel.colwise().norm();
	Eigen::RowVectorXf dist2 = dist.cwiseProduct(dist);
	Eigen::RowVectorXf dist3 = dist2.cwiseProduct(dist);
	Eigen::MatrixXf dists(arfCoefficient.cols(), dist.cols());
	dists << Eigen::RowVectorXf::Ones(dist.cols()), dist, dist2, dist3;
	return posRel.colwise().normalized() * (arfCoefficient * dists).asDiagonal();
}

Eigen::MatrixXf arfModelExperimentalPoly::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) {
	return arf(posRel, Eigen::MatrixXf());
}

arfModelFocusOnSphereExperimental::arfModelFocusOnSphereExperimental()
{
	this->tableDistance = Eigen::VectorXf::LinSpaced(10, 200, 2000);
	this->tableAngle = Eigen::VectorXf::LinSpaced(5, 0, M_PI / 3);
	this->tableARF.resize(5, 10);
	this->tableARF << 8.77695175, 7.7668668, 6.16838285, 4.79545185, 3.79517355, 2.97141495, 2.28494945, 1.8044236, 1.36312435, 1.0983448,
		7.6099604, 6.5704555, 5.22694445, 3.91285335, 3.01064155, 2.44185585, 1.93191005, 1.5886773, 1.48080415, 1.1964113,
		4.5698989, 3.5500073, 3.0204482, 2.44185585, 2.04958985, 1.5102241, 1.32389775, 1.12776475, 0.9610517, 0.77472535,
		3.08909475, 2.8047019, 2.4712758, 2.06920315, 1.67693715, 1.22583125, 1.06892485, 0.89240515, 0.7060788, 0.588399,
		1.67693715, 1.36312435, 1.1964113, 0.9414384, 0.7649187, 0.61781895, 0.4903325, 0.40207265, 0.36284605, 0.28439285;
}

Eigen::MatrixXf arfModelFocusOnSphereExperimental::arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd) {
	Eigen::RowVectorXf dists = posRel.colwise().norm();
	Eigen::RowVectorXf altitudes = posRel.cwiseProduct(directionsAutd.colwise().normalized()).colwise().sum();
	Eigen::RowVectorXf angles = altitudes.cwiseQuotient(dists).array().acos().matrix();
	Eigen::RowVectorXi indexesDist = (dists.replicate(tableDistance.rows(), 1) - tableDistance.replicate(1, dists.cols())).cwiseSign().cwiseMax(0).colwise().sum().cast<int>();
	indexesDist = (indexesDist.array() - 1).cwiseMax(0).cwiseMin(tableDistance.rows() - 2); // correct index outside the table
	Eigen::RowVectorXi indexesAngle = (angles.replicate(tableAngle.rows(), 1) - tableAngle.replicate(1, angles.cols())).cwiseSign().cwiseMax(0).colwise().sum().cast<int>();
	indexesAngle = (indexesAngle.array() - 1).cwiseMax(0).cwiseMin(tableAngle.rows() - 2);
	Eigen::RowVectorXf forces(posRel.cols());

	for (int i = 0; i < posRel.cols(); i++)
	{
		float f00 = tableARF(indexesAngle[i], indexesDist[i]);
		float f10 = tableARF(indexesAngle[i], indexesDist[i] + 1);
		float f01 = tableARF(indexesAngle[i] + 1, indexesDist[i]);
		float f11 = tableARF(indexesAngle[i] + 1, indexesDist[i] + 1);
		float r0 = tableDistance[indexesDist[i]];
		float r1 = tableDistance[indexesDist[i] + 1];
		float t0 = tableAngle[indexesAngle[i]];
		float t1 = tableAngle[indexesAngle[i] + 1];
		float dr = (dists[i] - r0) / (r1 - r0);
		float dt = (angles[i] - t0) / (t1 - t0);
		float f = f00 * (1 - dr) * (1 - dt) + f10 * dr * (1 - dt) + f01 * (1 - dr) * dt + f11 * dr * dt;
		forces[i] = f;
	}
	return posRel.colwise().normalized() * forces.asDiagonal(); // [mN]
}

Eigen::MatrixXf arfModelFocusOnSphereExperimental::arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf &eulerAnglesAUTD)
{
	Eigen::MatrixXf directionsAUTD(3, eulerAnglesAUTD.cols());
	for (int i = 0; i < directionsAUTD.cols(); i++)
	{
		directionsAUTD.col(i) <<
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(posRel, directionsAUTD);
}

Eigen::MatrixXf arfModelFocusOnSphereExperimental::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) {
	Eigen::Matrix3Xf directionsAutd(3, posRel.cols());
	for (int i_autd = 0; i_autd < posRel.cols(); i_autd++) {
		directionsAutd.col(i_autd) = rots[i_autd] * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(posRel, directionsAutd);
}


arfModelFocusSphereExp50mm::arfModelFocusSphereExp50mm(
) {
	this->tableDistance = Eigen::VectorXf::LinSpaced(20, 100, 1050);
	this->tableAngle = Eigen::VectorXf::LinSpaced(5, 0, M_PI / 3);
	this->tableArf.resize(5, 20);
	this->tableArf <<
		3.7828, 4.4394, 5.0862, 5.1548, 5.2577, 4.8412, 4.5864, 4.0474, 3.8661, 3.528, 3.3222, 2.9547, 2.6362, 2.2687, 2.1364, 1.8375, 1.6954, 1.3818, 1.2936, 1.127,
		3.4594, 4.3022, 4.4737, 4.4541, 4.4198, 4.1013, 3.9151, 3.5819, 3.3124, 3.0576, 2.8959, 2.6362, 2.4794, 2.2589, 2.1609, 1.911, 1.7591, 1.5386, 1.5043, 1.4014,
		2.9939, 3.283, 3.332, 2.94, 2.8469, 2.4745, 2.3961, 2.1609, 1.9649, 1.8473, 1.6856, 1.6023, 1.4847, 1.4112, 1.3181, 1.1907, 1.1074, 0.9898, 0.931, 0.8624,
		2.2246, 2.5088, 2.3128, 2.0678, 1.9355, 1.8473, 1.7934, 1.7395, 1.666, 1.5729, 1.47, 1.3916, 1.2838, 1.1809, 1.0682, 0.9751, 0.882, 0.8232, 0.735, 0.6664,
		1.35, 1.5876, 1.4161, 1.2593, 1.1319, 1.0437, 0.9947, 0.8918, 0.8232, 0.7399, 0.6909, 0.6174, 0.5733, 0.5145, 0.4655, 0.4214, 0.3724, 0.3332, 0.294, 0.2842;

		//7.5656, 8.8788, 10.1724, 10.3096, 10.5154, 9.6824, 9.1728, 8.0948, 7.7322, 7.056, 6.6444, 5.9094, 5.2724, 4.5374, 4.2728, 3.675, 3.3908, 2.7636, 2.5872, 2.254,
		//6.9188, 8.6044, 8.9474, 8.9082, 8.8396, 8.2026, 7.8302, 7.1638, 6.6248, 6.1152, 5.7918, 5.2724, 4.9588, 4.5178, 4.3218, 3.822, 3.5182, 3.0772, 3.0086, 2.8028,
		//5.9878, 6.566, 6.664, 5.88, 5.6938, 4.949, 4.7922, 4.3218, 3.9298, 3.6946, 3.3712, 3.2046, 2.9694, 2.8224, 2.6362, 2.3814, 2.2148, 1.9796, 1.862, 1.7248,
		//4.4492, 5.0176, 4.6256, 4.1356, 3.871, 3.6946, 3.5868, 3.479, 3.332, 3.1458, 2.94, 2.7832, 2.5676, 2.3618, 2.1364, 1.9502, 1.764, 1.6464, 1.47, 1.3328,
		//2.7, 3.1752, 2.8322, 2.5186, 2.2638, 2.0874, 1.9894, 1.7836, 1.6464, 1.4798, 1.3818, 1.2348, 1.1466, 1.029, 0.931, 0.8428, 0.7448, 0.6664, 0.588, 0.5684;
}

Eigen::MatrixXf arfModelFocusSphereExp50mm::arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd) {
	Eigen::RowVectorXf dists = posRel.colwise().norm();
	Eigen::RowVectorXf altitudes = posRel.cwiseProduct(directionsAutd.colwise().normalized()).colwise().sum();
	Eigen::RowVectorXf angles = altitudes.cwiseQuotient(dists).cwiseMin(1.0f).cwiseMax(-1.0f).array().acos().matrix();
	Eigen::RowVectorXi indexesDist = (dists.replicate(tableDistance.rows(), 1) - tableDistance.replicate(1, dists.cols())).cwiseSign().cwiseMax(0).colwise().sum().cast<int>();
	indexesDist = (indexesDist.array() - 1).cwiseMax(0).cwiseMin(tableDistance.rows() - 2); // correct index outside the table
	Eigen::RowVectorXi indexesAngle = (angles.replicate(tableAngle.rows(), 1) - tableAngle.replicate(1, angles.cols())).cwiseSign().cwiseMax(0).colwise().sum().cast<int>();
	indexesAngle = (indexesAngle.array() - 1).cwiseMax(0).cwiseMin(tableAngle.rows() - 2);
	Eigen::RowVectorXf forces(posRel.cols());

	for (int i = 0; i < posRel.cols(); i++)
	{
		float f00 = tableArf(indexesAngle[i], indexesDist[i]);
		float f10 = tableArf(indexesAngle[i], indexesDist[i] + 1);
		float f01 = tableArf(indexesAngle[i] + 1, indexesDist[i]);
		float f11 = tableArf(indexesAngle[i] + 1, indexesDist[i] + 1);
		float r0 = tableDistance[indexesDist[i]];
		float r1 = tableDistance[indexesDist[i] + 1];
		float t0 = tableAngle[indexesAngle[i]];
		float t1 = tableAngle[indexesAngle[i] + 1];
		float dr = (dists[i] - r0) / (r1 - r0);
		float dt = (angles[i] - t0) / (t1 - t0);
		float f = f00 * (1 - dr) * (1 - dt) + f10 * dr * (1 - dt) + f01 * (1 - dr) * dt + f11 * dr * dt;
		forces[i] = f;
	}
	return posRel.colwise().normalized() * forces.asDiagonal(); // [mN]
}

Eigen::MatrixXf arfModelFocusSphereExp50mm::arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD)
{
	Eigen::MatrixXf directionsAUTD(3, eulerAnglesAUTD.cols());
	for (int i = 0; i < directionsAUTD.cols(); i++)
	{
		directionsAUTD.col(i) <<
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(posRel, directionsAUTD);
}

Eigen::MatrixXf arfModelFocusSphereExp50mm::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots)
{
	Eigen::Matrix3Xf directionsAutd(3, posRel.cols());
	for (int i_autd = 0; i_autd < posRel.cols(); i_autd++) {
		directionsAutd.col(i_autd) = rots[i_autd] * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(posRel, directionsAutd);
}


#endif