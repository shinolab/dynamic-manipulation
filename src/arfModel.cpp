#ifndef _ARF_MODEL_
#define _ARF_MODEL_

#include "arfModel.hpp"
#include "read-csv-to-eigen.hpp"
#include <iostream>
#include <Eigen\Dense>

#define NUM_TABLE_OFFSET 21
#define NUM_TABLE_DISTANCE 8

namespace {
	const Eigen::Matrix<float, NUM_TABLE_DISTANCE, 1> tableDistance = []{
		Eigen::Matrix<float, NUM_TABLE_DISTANCE, 1> tD;
		tD.setLinSpaced(400, 1800);
		return tD;
	}();

	const Eigen::Matrix<float, NUM_TABLE_OFFSET, 1> tableOffset = []{
		Eigen::Matrix<float, NUM_TABLE_OFFSET, 1> tO;
		tO.setLinSpaced(0, 200);
		return tO;
	}();

	const Eigen::MatrixXf tableArfX = readCSV("arfX-smooth.csv", 8, 21);
	
	const Eigen::MatrixXf tableArfZ = readCSV("arfZ-smooth.csv", 8, 21);
}

//return perpendicular componet of ARF [mN]
float arfModel::arfX(float distance, float offset)
{
	int iDist = (distance * tableDistance.Ones() - tableDistance).cwiseSign().cwiseMax(0).sum() - 1;
	int iOffset = (offset * tableOffset.Ones() - tableOffset).cwiseSign().cwiseMax(0).sum() - 1;
	iDist = (iDist < 0) ? iDist += 1 : iDist; (iDist >= tableDistance.cols() - 1) ? iDist -= 1 : iDist;
	iOffset = (iOffset < 0) ? iOffset += 1 : iOffset; (iOffset >= tableOffset.cols() - 1) ? iOffset -= 1 : iOffset;
	float intervalDist = tableDistance[iDist + 1] - tableDistance[iDist];
	float intervalOffset = tableOffset[iOffset + 1] - tableOffset[iOffset];
	float dz = (distance - tableDistance[iDist]) / intervalDist;
	float dx = (offset - tableOffset[iOffset]) / intervalOffset;
	float fx = tableArfX(iDist, iOffset) * (1 - dx) * (1 - dz)
		+ tableArfX(iDist + 1, iOffset) * dz * (1 - dx)
		+ tableArfX(iDist, iOffset + 1) * (1 - dz) * dx
		+ tableArfX(iDist + 1, iOffset + 1) * dx * dx;
	return std::max(fx, (float)0);
}

float arfModel::arfZ(float distance, float offset)
{
	int iDist = (distance * tableDistance.Ones() - tableDistance).cwiseSign().cwiseMax(0).sum() - 1;
	int iOffset = (offset * tableOffset.Ones() - tableOffset).cwiseSign().cwiseMax(0).sum() - 1;
	iDist = (iDist < 0) ? iDist += 1 : iDist; (iDist >= tableDistance.cols() - 1) ? iDist -= 1 : iDist;
	iOffset = (iOffset < 0) ? iOffset += 1 : iOffset; (iOffset >= tableOffset.cols() - 1) ? iOffset -= 1 : iOffset;
	float intervalDist = tableDistance[iDist + 1] - tableDistance[iDist];
	float intervalOffset = tableOffset[iOffset + 1] - tableOffset[iOffset];
	float dz = (distance - tableDistance[iDist]) / intervalDist;
	float dx = (offset - tableOffset[iOffset]) / intervalOffset;
	float fz = tableArfZ(iDist, iOffset) * (1 - dx) * (1 - dz)
		+ tableArfZ(iDist + 1, iOffset) * dz * (1 - dx)
		+ tableArfZ(iDist, iOffset + 1) * (1 - dz) * dx
		+ tableArfZ(iDist + 1, iOffset + 1) * dx * dx;
	return std::max(fz, (float)0);
}

//Return 3-by-(numAUTD) matrix where each column represents ARF by AUTD at muximum duty. 
Eigen::MatrixXf arfModel::arf(Eigen::MatrixXf posRel)
{
	posRel /= 1000;
	Eigen::RowVector4f arfCoefficient;
	arfCoefficient << 0.3683, 0.6913, -0.9431, 0.2769; arfCoefficient *= 9.8; // [mN]
	Eigen::RowVectorXf dist = posRel.colwise().norm();
	Eigen::RowVectorXf dist2 = dist.cwiseProduct(dist);
	Eigen::RowVectorXf dist3 = dist2.cwiseProduct(dist);
	Eigen::MatrixXf dists(arfCoefficient.cols(), dist.cols());
	dists << Eigen::RowVectorXf::Ones(dist.cols()), dist, dist2, dist3;
	return posRel.colwise().normalized() * (arfCoefficient * dists).asDiagonal();
}

Eigen::MatrixXf arfModel::arfDirections(Eigen::MatrixXf posRel, Eigen::VectorXf duties, Eigen::MatrixXf directions)
{
	Eigen::MatrixXf forces(posRel.rows(), posRel.cols());
	directions.colwise().normalize();
	for (int i = 0; i < directions.cols(); i++)
	{
		Eigen::Vector3f offset = (posRel.col(i).norm() / directions.col(i).dot(posRel.col(i).normalized())) * directions.col(i) - posRel.col(i);
		float distance = posRel.col(i).norm();
		int iDist = (distance * tableDistance.Ones() - tableDistance).cwiseSign().cwiseMax(0).sum() - 1;
		int iOffset = (offset.norm() * tableOffset.Ones() - tableOffset).cwiseSign().cwiseMax(0).sum() - 1;
		iDist = (iDist < 0) ? iDist += 1 : iDist; (iDist >= tableDistance.rows() - 1) ? iDist -= 1 : iDist;
		iOffset = (iOffset < 0) ? iOffset += 1 : iOffset; (iOffset >= tableOffset.rows() - 1) ? iOffset -= 1 : iOffset;
		float intervalDist = tableDistance[iDist + 1] - tableDistance[iDist];
		float intervalOffset = tableOffset[iOffset + 1] - tableOffset[iOffset];
		float dz = (distance - tableDistance[iDist]) / intervalDist;
		float dx = (offset.norm() - tableOffset[iOffset]) / intervalOffset;
		float fx = tableArfX(iDist, iOffset) * (1 - dx) * (1 - dz)
			+ tableArfX(iDist + 1, iOffset) * dz * (1 - dx)
			+ tableArfX(iDist, iOffset + 1) * (1 - dz) * dx
			+ tableArfX(iDist + 1, iOffset + 1) * dz * dx;
		float fz = tableArfZ(iDist, iOffset) * (1 - dx) * (1 - dz)
			+ tableArfZ(iDist + 1, iOffset) * dz * (1 - dx)
			+ tableArfZ(iDist, iOffset + 1) * (1 - dz) * dx
			+ tableArfZ(iDist + 1, iOffset + 1) * dz * dx;
		forces.col(i) = duties[i] * (std::max(fz, (float)0) * posRel.col(i).normalized() -std::max(fx, (float)0) * offset.normalized());
	}
	return forces;
}

Eigen::Vector3f arfModel::arfDirectionsTotal(Eigen::MatrixXf posRel, Eigen::VectorXf duties, Eigen::MatrixXf directions)
{
	return arfDirections(posRel, duties, directions).rowwise().sum();
}

Eigen::MatrixXf arfModel::arfOffsets(const Eigen::MatrixXf posRel, const Eigen::VectorXf duties, const Eigen::MatrixXf offsets)
{
	Eigen::MatrixXf directions = (posRel + offsets).colwise().normalized();
	return arfModel::arfDirections(posRel, duties, directions);
	/*
	Eigen::MatrixXf forces(posRel.rows(), posRel.cols());
	for (int i = 0; i < posRel.cols(); i++)
	{
	float distance = posRel.col(i).norm();
	Eigen::Vector3f offset = offsets.col(i) - offsets.col(i).dot(posRel.col(i).normalized())*(offsets.col(i).normalized());
	int iDist = (distance * tableDistance.Ones() - tableDistance).cwiseSign().cwiseMax(0).sum() - 1;
	int iOffset = (offset.norm() * tableOffset.Ones() - tableOffset).cwiseSign().cwiseMax(0).sum() - 1;
	iDist = (iDist < 0) ? iDist += 1 : iDist; (iDist >= tableDistance.rows() - 1) ? iDist -= 1 : iDist;
	iOffset = (iOffset < 0) ? iOffset += 1 : iOffset; (iOffset >= tableOffset.rows() - 1) ? iOffset -= 1 : iOffset;
	float intervalDist = tableDistance[iDist + 1] - tableDistance[iDist];
	float intervalOffset = tableOffset[iOffset + 1] - tableOffset[iOffset];
	float dz = (distance - tableDistance[iDist]) / intervalDist;
	float dx = (offset.norm() - tableOffset[iOffset]) / intervalOffset;
	float fx = tableArfX(iDist, iOffset) * (1 - dx) * (1 - dz)
	+ tableArfX(iDist + 1, iOffset) * dz * (1 - dx)
	+ tableArfX(iDist, iOffset + 1) * (1 - dz) * dx
	+ tableArfX(iDist + 1, iOffset + 1) * dz * dx;
	float fz = tableArfZ(iDist, iOffset) * (1 - dx) * (1 - dz)
	+ tableArfZ(iDist + 1, iOffset) * dz * (1 - dx)
	+ tableArfZ(iDist, iOffset + 1) * (1 - dz) * dx
	+ tableArfZ(iDist + 1, iOffset + 1) * dz * dx;
	forces.col(i)= duties[i] * (std::max(fz, (float)0) * posRel.col(i).normalized() - std::max(fx, (float)0) * offset.normalized());
	}
	return forces;
	*/
	
}

Eigen::Vector3f arfModel::arfTotalOffsets(const Eigen::MatrixXf posRel, const Eigen::VectorXf duties, const Eigen::MatrixXf offsets)
{
	return arfModel::arfOffsets(posRel, duties, offsets).rowwise().sum();
}

//Z component of offsets must be zero.
Eigen::MatrixXf arfModel::arfOffsetsLocal(const Eigen::MatrixXf posRel, const Eigen::MatrixXf eulerAnglesAUTD, const Eigen::VectorXf duties, const Eigen::MatrixXf offsetsLocal)
{
	Eigen::MatrixXf forces(posRel.rows(), posRel.cols());
	for (int i = 0; i < posRel.cols(); i++)
	{
		Eigen::Vector3f offsetL = offsetsLocal.col(i);
		float distance = posRel.col(i).norm();
		int iDist = (distance * tableDistance.Ones() - tableDistance).cwiseSign().cwiseMax(0).sum() - 1;
		int iOffset = (offsetL.norm() * tableOffset.Ones() - tableOffset).cwiseSign().cwiseMax(0).sum() - 1;
		iDist = (iDist < 0) ? iDist += 1 : iDist; (iDist >= tableDistance.rows() - 1) ? iDist -= 1 : iDist;
		iOffset = (iOffset < 0) ? iOffset += 1 : iOffset; (iOffset >= tableOffset.rows() - 1) ? iOffset -= 1 : iOffset;
		float intervalDist = tableDistance[iDist + 1] - tableDistance[iDist];
		float intervalOffset = tableOffset[iOffset + 1] - tableOffset[iOffset];
		float dz = (distance - tableDistance[iDist]) / intervalDist;
		float dx = (offsetL.norm() - tableOffset[iOffset]) / intervalOffset;
		float fx = tableArfX(iDist, iOffset) * (1 - dx) * (1 - dz)
			+ tableArfX(iDist + 1, iOffset) * dz * (1 - dx)
			+ tableArfX(iDist, iOffset + 1) * (1 - dz) * dx
			+ tableArfX(iDist + 1, iOffset + 1) * dz * dx;
		float fz = tableArfZ(iDist, iOffset) * (1 - dx) * (1 - dz)
			+ tableArfZ(iDist + 1, iOffset) * dz * (1 - dx)
			+ tableArfZ(iDist, iOffset + 1) * (1 - dz) * dx
			+ tableArfZ(iDist + 1, iOffset + 1) * dz * dx;
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		forces.col(i) = quo.inverse() * (duties[i] * (std::max(fz, (float)0) * posRel.col(i).normalized() - std::max(fx, (float)0) * offsetL.normalized()));
	}
	return forces;

}

Eigen::Vector3f arfModel::arfTotalOffsetsLocal(const Eigen::MatrixXf posRel, const Eigen::MatrixXf eulerAnglesAUTDS, const Eigen::VectorXf duties, const Eigen::MatrixXf offsetsLocal)
{
	return arfTotalOffsetsLocal(posRel, eulerAnglesAUTDS, duties, offsetsLocal).rowwise().sum();
}

#endif