#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <memory>
#include <Eigen\Dense>

class arfModelLinearBase
{
public:
	virtual Eigen::MatrixXf arf(Eigen::MatrixXf const &posRel, Eigen::MatrixXf const &eulerAnglesAUTD) = 0;
};

class arfModelConstant : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf const &posRel, Eigen::MatrixXf const &eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
};

class arfModelExperimentalPoly : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf const &posRel, Eigen::MatrixXf const &eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
};

class arfModelTheoreticalTable : public arfModelLinearBase
{
public:
	arfModelTheoreticalTable();
	Eigen::MatrixXf arf(const Eigen::MatrixXf &_posRel, const Eigen::MatrixXf &_eulerAnglesAUTD) override;

	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableARF;
};

class arfModelFocusOnSphereExperimental : public arfModelLinearBase
{
public:
	arfModelFocusOnSphereExperimental();
	Eigen::MatrixXf arf(const Eigen::MatrixXf &_posRel, const Eigen::MatrixXf &_eulerAnglesAUTD);
	
private:
	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableARF;
};

namespace arfModel
{
	Eigen::MatrixXf arf(Eigen::MatrixXf const &posRel);
}

#endif // !ARFMODEL_H_