#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <memory>
#include <vector>
#include <Eigen\Dense>

class arfModelLinearBase
{
public:
	virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD) = 0;
	virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) = 0;
};

class arfModelConstant : public arfModelLinearBase
{
private:
	float m_intensity;
public:
	arfModelConstant(float intensity = 5.012);
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
};

class arfModelExperimentalPoly : public arfModelLinearBase
{
	const Eigen::RowVector4f m_coeffs;
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf const &posRel, Eigen::MatrixXf const &eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
};

class arfModelTheoreticalTable : public arfModelLinearBase
{
public:
	arfModelTheoreticalTable();
	Eigen::MatrixXf arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd);
	Eigen::MatrixXf arf(const Eigen::MatrixXf &_posRel, const Eigen::MatrixXf &_eulerAnglesAUTD) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableARF;
};

class arfModelFocusOnSphereExperimental : public arfModelLinearBase
{
public:
	arfModelFocusOnSphereExperimental();
	Eigen::MatrixXf arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd);
	Eigen::MatrixXf arf(const Eigen::MatrixXf &_posRel, const Eigen::MatrixXf &_eulerAnglesAUTD) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
	
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