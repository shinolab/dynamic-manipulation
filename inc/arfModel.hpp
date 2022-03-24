#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <memory>
#include <vector>
#include <Eigen\Dense>

class arfModelLinearBase
{
public:
	~arfModelLinearBase();
	virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD) = 0;
	virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) = 0;
};

class arfModelConstant : public arfModelLinearBase
{
private:
	float m_intensity;
public:
	arfModelConstant(float intensity = 5.012f);
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
};

class arfModelExperimentalPoly : public arfModelLinearBase
{
	const Eigen::RowVector4f m_coeffs;
public:
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD = Eigen::Matrix3f::Zero()) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
};


class arfModelFocusOnSphereExperimental : public arfModelLinearBase
{
public:
	arfModelFocusOnSphereExperimental();
	Eigen::MatrixXf arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd);
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf &eulerAnglesAUTD) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
	
private:
	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableARF;
};

class arfModelFocusSphereExp50mm : public arfModelLinearBase {
public:
	arfModelFocusSphereExp50mm();
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAngles) override;
	Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
	Eigen::MatrixXf arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd);

private:
	Eigen::VectorXf tableDistance;
	Eigen::VectorXf tableAngle;
	Eigen::MatrixXf tableArf;
};


#endif // !ARFMODEL_H_