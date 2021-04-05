#ifndef _ARF_MODEL_
#define _ARF_MODEL_

#include "arfModel.hpp"
#include "read-csv-to-eigen.hpp"
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

arfModelTheoreticalTable::arfModelTheoreticalTable()
{
	this->tableDistance = Eigen::VectorXf::LinSpaced(8, 0.4, 1.8);
	this->tableAngle = Eigen::VectorXf::LinSpaced(6, 0, 5.0*M_PI / 9.0);
	this->tableARF.resize(8, 6);
	this->tableARF << 
		1.000000000, 1.025280603, 1.104517639, 1.249525661, 1.481451134, 1.849174802,
		0.986364501, 1.010438669, 1.085707808, 1.221311067, 1.437984013, 1.777854586,
		0.972982686, 0.995547584, 1.066292486, 1.194949282, 1.398205465, 1.707827359,
		0.953808427, 0.975389474, 1.042081835, 1.160212992, 1.344836418, 1.632654528,
		0.933133205, 0.952746075, 1.013621177, 1.123370632, 1.299178981, 1.569093517,
		0.911749783, 0.930658413, 0.989791487, 1.096874291, 1.264885618, 1.506332866,
		0.892205568, 0.910729752, 0.968241851, 1.070133146, 1.222860568, 1.427376353,
		0.869627719, 0.887113992, 0.940677195, 1.032894236, 1.165005918, 1.331883179;
	this->tableARF *= 5.012;
}

Eigen::MatrixXf arfModelTheoreticalTable::arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd) {
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
		float f00 = tableARF(indexesDist[i], indexesAngle[i]);
		float f10 = tableARF(indexesDist[i] + 1, indexesAngle[i]);
		float f01 = tableARF(indexesDist[i], indexesAngle[i] + 1);
		float f11 = tableARF(indexesDist[i] + 1, indexesAngle[i] + 1);
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

Eigen::MatrixXf arfModelTheoreticalTable::arf(const Eigen::MatrixXf& _posRel, const Eigen::MatrixXf& eulerAnglesAUTD)
{
	Eigen::MatrixXf posRel = _posRel/1000.f;
	Eigen::MatrixXf directionsAUTD(3, eulerAnglesAUTD.cols());
	for (int i = 0; i < directionsAUTD.cols(); i++)
	{
		directionsAUTD.col(i) << 
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(_posRel, directionsAUTD);
}

Eigen::MatrixXf arfModelTheoreticalTable::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) {
	Eigen::Matrix3Xf directionsAutd(3, posRel.cols());
	for (int i_autd = 0; i_autd < posRel.cols(); i_autd++) {
		directionsAutd.col(i_autd) = rots[i_autd] * Eigen::Vector3f::UnitZ();
	}
	return arfFromDirections(posRel, directionsAutd);
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

arfModelFocusOnSphereD5::arfModelFocusOnSphereD5() {
	const int numDist = 11;
	const int numAngle = 9;
	this->tableDistance = Eigen::VectorXf::LinSpaced(numDist, 100, 600);
	this->tableAngle = Eigen::VectorXf::LinSpaced(numAngle, 0, 4 * M_PI / 9);
	this->tableArfVertical.resize(numDist, numAngle);
	this->tableArfHorizontal.resize(numDist, numAngle);
	this->tableArfVertical <<
		1.90E-02, 1.84E-02, 1.69E-02, 1.46E-02, 1.17E-02, 8.71E-03, 6.55E-03, 0.004348326, 2.32E-03,
		2.37E-02, 2.28E-02, 2.04E-02, 1.70E-02, 1.30E-02, 8.54E-03, 4.60E-03, 0.002148835, 9.43E-04,
		2.47E-02, 2.40E-02, 2.16E-02, 1.78E-02, 1.29E-02, 7.61E-03, 3.73E-03, 0.001697458, 9.61E-04,
		2.45E-02, 2.41E-02, 2.21E-02, 1.81E-02, 1.25E-02, 6.93E-03, 3.33E-03, 0.001527912, 9.90E-04,
		2.39E-02, 2.38E-02, 2.23E-02, 1.82E-02, 1.20E-02, 6.48E-03, 3.07E-03, 0.001438716, 8.79E-04,
		2.34E-02, 2.34E-02, 2.22E-02, 1.80E-02, 1.15E-02, 6.15E-03, 2.86E-03, 0.001373089, 6.74E-04,
		2.29E-02, 2.29E-02, 2.20E-02, 1.78E-02, 1.11E-02, 5.85E-03, 2.73E-03, 0.001283384, 5.60E-04,
		2.24E-02, 2.24E-02, 2.17E-02, 1.76E-02, 1.07E-02, 5.60E-03, 2.62E-03, 0.001182336, 4.71E-04,
		2.18E-02, 2.18E-02, 2.13E-02, 1.72E-02, 1.03E-02, 5.39E-03, 2.50E-03, 0.001085084, 4.12E-04,
		2.13E-02, 2.13E-02, 2.09E-02, 1.68E-02, 9.95E-03, 5.20E-03, 2.38E-03, 0.00099765, 3.65E-04,
		2.08E-02, 2.07E-02, 2.04E-02, 1.64E-02, 9.65E-03, 5.02E-03, 2.26E-03, 0.000920678, 3.27E-04;
	this->tableArfHorizontal <<
		4.78E-05, 0.001683716, 0.003144588, 0.004250118, 0.00484127, 0.004922806, 4.88E-03, 4.98E-03, 5.06E-03,
		6.25E-05, 0.003153982, 0.00554344, 0.007020677, 0.007508789, 0.006992697, 5.71E-03, 4.52E-03, 4.42E-03,
		5.75E-05, 0.003933318, 0.00684551, 0.008441441, 0.00862748, 0.007381622, 5.52E-03, 4.17E-03, 4.30E-03,
		4.54E-05, 0.004161848, 0.007476261, 0.009168828, 0.009049355, 7.32E-03, 5.28E-03, 3.95E-03, 4.05E-03,
		3.53E-05, 4.16E-03, 7.77E-03, 9.52E-03, 9.14E-03, 7.16E-03, 5.07E-03, 3.76E-03, 3.72E-03,
		2.82E-05, 4.09E-03, 7.89E-03, 9.71E-03, 9.06E-03, 6.98E-03, 4.88E-03, 3.57E-03, 3.36E-03,
		2.31E-05, 4.00E-03, 7.88E-03, 9.75E-03, 8.91E-03, 6.80E-03, 4.69E-03, 3.41E-03, 3.03E-03,
		1.91E-05, 3.90E-03, 7.79E-03, 9.71E-03, 8.73E-03, 6.58E-03, 4.51E-03, 3.24E-03, 2.73E-03,
		1.60E-05, 3.81E-03, 7.67E-03, 9.63E-03, 8.50E-03, 6.36E-03, 4.35E-03, 3.05E-03, 2.47E-03,
		1.35E-05, 3.72E-03, 7.53E-03, 9.49E-03, 8.25E-03, 6.16E-03, 4.19E-03, 2.87E-03, 2.24E-03,
		1.17E-05, 3.63E-03, 7.37E-03, 9.33E-03, 8.03E-03, 5.98E-03, 4.03E-03, 2.69E-03, 2.04E-03;			
}

Eigen::MatrixXf arfModelFocusOnSphereD5::arf(
	const Eigen::MatrixXf& posRel,
	const Eigen::MatrixXf& eulerAngles
) {
	std::vector<Eigen::Matrix3f> rots(eulerAngles.cols());
	for (int iAupa = 0; iAupa < eulerAngles.cols(); iAupa++) {
		rots[iAupa] 
			= Eigen::AngleAxisf(eulerAngles.col(iAupa).x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(eulerAngles.col(iAupa).y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(eulerAngles.col(iAupa).z(), Eigen::Vector3f::UnitZ());
	}
	return arf(posRel, rots);
}

Eigen::MatrixXf arfModelFocusOnSphereD5::arf(
	const Eigen::MatrixXf& posRel,
	const std::vector<Eigen::Matrix3f>& rots
) {
	Eigen::MatrixXf directionsAutd(3, rots.size());
	for (int i_aupa = 0; i_aupa < rots.size(); i_aupa++) {
		directionsAutd.col(i_aupa) = rots[i_aupa] * Eigen::Vector3f::UnitZ();
	}
	Eigen::VectorXf dists = posRel.colwise().norm();
	Eigen::VectorXf heights = posRel.cwiseProduct(directionsAutd.colwise().normalized()).colwise().sum().transpose();
	Eigen::VectorXf angles = heights.cwiseQuotient(dists).array().acos().matrix().transpose()	;
	Eigen::MatrixXf arfMatrix(3, posRel.cols());
	for (int i_aupa = 0; i_aupa < posRel.cols(); i_aupa++) {
		auto forceVertical = linearInterp2d(dists[i_aupa], angles[i_aupa], tableDistance, tableAngle, tableArfVertical);
		auto forceHorizontal = linearInterp2d(dists[i_aupa], angles[i_aupa], tableDistance, tableAngle, tableArfHorizontal);
		arfMatrix.col(i_aupa) = rots[i_aupa] * Eigen::Vector3f(forceHorizontal, 0.f, forceVertical);
	}
	return arfMatrix;
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
	Eigen::RowVectorXf angles = altitudes.cwiseQuotient(dists).array().acos().matrix();
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

//Return 3-by-(numAUTD) matrix where each column represents ARF by AUTD at muximum duty. 
Eigen::MatrixXf arfModel::arf(const Eigen::MatrixXf& _posRel)
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

#endif