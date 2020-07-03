#ifndef _ARF_MODEL_
#define _ARF_MODEL_

#include "arfModel.hpp"
#include "read-csv-to-eigen.hpp"
#include <Eigen\Dense>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

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
	//std::cout << "directionsAUTD\n" << directionsAUTD << std::endl;

	return arfFromDirections(posRel, directionsAUTD);
	
}

Eigen::MatrixXf arfModelFocusOnSphereExperimental::arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) {
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