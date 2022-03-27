#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <memory>
#include <vector>
#include <Eigen\Dense>

namespace dynaman {
	class arfModelLinearBase
	{
	public:
		~arfModelLinearBase();
		virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAUTD) = 0;
		virtual Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) = 0;
	};

	class arfModelTabular : public arfModelLinearBase
	{
	private:
		Eigen::VectorXf m_tableDistance;
		Eigen::VectorXf m_tableAngle;
		Eigen::MatrixXf m_tableForce;

		Eigen::MatrixXf arfFromDirections(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& directionsAutd);

	public:
		arfModelTabular(const Eigen::VectorXf& tableDistance, const Eigen::VectorXf& tableAngle, const Eigen::MatrixXf& tableForce);
		Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const Eigen::MatrixXf& eulerAnglesAutd) override;
		Eigen::MatrixXf arf(const Eigen::MatrixXf& posRel, const std::vector<Eigen::Matrix3f>& rots) override;
	};

	class arfModelFocusSphereR100 : public arfModelTabular
	{
	public:
		arfModelFocusSphereR100();

	private:
		Eigen::VectorXf tableDistances() const;
		Eigen::VectorXf tableAngles() const;
		Eigen::MatrixXf tableForces() const;
	};

	class arfModelFocusSphereR50 : public arfModelTabular {
	public:
		arfModelFocusSphereR50();

	private:
		Eigen::VectorXf tableDistances() const;
		Eigen::VectorXf tableAngles() const;
		Eigen::MatrixXf tableForces() const;
	};
}


#endif // !ARFMODEL_H_