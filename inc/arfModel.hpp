#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <memory>
#include <Eigen\Dense>

class arfModelLinearBase
{
public:
	virtual Eigen::MatrixXf arf(Eigen::MatrixXf posRel, Eigen::MatrixXf eulerAnglesAUTD) = 0;
};

class arfModelConstant : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel, Eigen::MatrixXf eulerAnglesAUTD = Eigen::Matrix3f::Zero());
};

class arfModelExperimentalPoly : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel, Eigen::MatrixXf eulerAnglesAUTD = Eigen::Matrix3f::Zero());
};

class arfModelTheoreticalTable : public arfModelLinearBase
{
public:
	arfModelTheoreticalTable();
	Eigen::MatrixXf arf(const Eigen::MatrixXf _posRel, const Eigen::MatrixXf _eulerAnglesAUTD);

	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableARF;
};

namespace arfModel
{
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
}

#endif // !ARFMODEL_H_