#include "odcs.hpp"
#include "autd3.hpp"
#include "arfModel.hpp"
#include "additionalGain.hpp"
#include "QPSolver.h"
#include <Eigen\Geometry>
#include <dlib\matrix.h>
#include <dlib\optimization.h>
#include <algorithm>
#include <vector>
#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>

#define NUM_AUTDS 5
#define NUM_STATES 6

int ocs::Initialize()
{
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) return ENXIO;

	Eigen::Vector3f positionAUTD0(-85, -65, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-998, 65, 1038); Eigen::Vector3f eulerAngleAUTD1(M_PI, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD2(998, -65, 1038); Eigen::Vector3f eulerAngleAUTD2(0, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD3(-65, -998, 1038); Eigen::Vector3f eulerAngleAUTD3(-M_PI_2, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD4(65, 998, 1038); Eigen::Vector3f eulerAngleAUTD4(M_PI_2, -M_PI_2, 0);

	autd.geometry()->AddDevice(positionAUTD0, eulerAngleAUTD0);
	autd.geometry()->AddDevice(positionAUTD1, eulerAngleAUTD1);
	autd.geometry()->AddDevice(positionAUTD2, eulerAngleAUTD2);
	autd.geometry()->AddDevice(positionAUTD3, eulerAngleAUTD3);
	autd.geometry()->AddDevice(positionAUTD4, eulerAngleAUTD4);
	
	positionsAUTD.resize(3, autd.geometry()->numDevices());
	positionsAUTD << positionAUTD0, positionAUTD1, positionAUTD2, positionAUTD3, positionAUTD4;
	eulerAnglesAUTD.resize(3, autd.geometry()->numDevices());
	eulerAnglesAUTD << eulerAngleAUTD0, eulerAngleAUTD1, eulerAngleAUTD2, eulerAngleAUTD3, eulerAngleAUTD4;
	centersAUTD.resize(3, autd.geometry()->numDevices());
	directionsAUTD.resize(3, autd.geometry()->numDevices());
	for (int i = 0; i < autd.geometry()->numDevices(); i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		Eigen::Affine3f affineGlobal2AUTD = Eigen::Translation3f(positionsAUTD.col(i)) * quo;
		centersAUTD.col(i) << affineGlobal2AUTD * Eigen::Vector3f(85, 65, 0);
		directionsAUTD.col(i) << quo * Eigen::Vector3f::UnitZ();
	}
	//arfModelPtr.reset(new arfModelTheoreticalTable());
	arfModelPtr.reset(new arfModelFocusOnSphereExperimental());
	std::cout << "centersAUTD\n" << centersAUTD << std::endl;
	return 0;
}

void ocs::Close()
{
	autd.Close();
	//engClose(ep);
}

void ocs::RegisterObject(FloatingObjectPtr objPtr)
{
	objPtr->inputLatest.resize(positionsAUTD.cols());
	objPtr->inputLatest.setZero();
}

void ocs::SetArfModel(std::unique_ptr<arfModelLinearBase> _arfModelPtr)
{
	this->arfModelPtr = std::move(_arfModelPtr);
}

void ocs::SetGain(Eigen::Vector3f const &_gainP, Eigen::Vector3f const &_gainD, Eigen::Vector3f const &_gainI)
{
	this->gainP = _gainP;
	this->gainD = _gainD;
	this->gainI = _gainI;
}

Eigen::Vector3f ocs::ComputePIDForce(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dv = objPtr->getVelocity() - objPtr->getVelocityTarget();
	Eigen::Vector3f acceleration = gainP.asDiagonal() * dr + gainD.asDiagonal() * dv + gainI.asDiagonal() * objPtr->getIntegral() + objPtr->getAccelTarget();
	Eigen::Vector3f force = objPtr->totalMass() * acceleration - objPtr->additionalMass * Eigen::Vector3f(0, 0, -9.80665e3f);
	return force;
}

autd::GainPtr ocs::CreateGain(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f accel
		= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
		+ gainD.asDiagonal() * (objPtr->getVelocity() - objPtr->getVelocityTarget())
		+ gainI.asDiagonal() * objPtr->getIntegral()
		+ objPtr->getAccelTarget();
	Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665f);
	Eigen::VectorXf duties = FindDutyQP(forceToApply, objPtr->getPosition());
	Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
	Eigen::MatrixXf focus = centersAUTD + (objPtr->getPosition().replicate(1, centersAUTD.cols()) - centersAUTD);
	return autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes);
}

void ocs::DirectSemiPlaneWave(FloatingObjectPtr objPtr, Eigen::VectorXi const &amplitudes)
{
	float outpor = 100;
	Eigen::MatrixXf farPoints = centersAUTD + outpor * (objPtr->getPosition().replicate(1, centersAUTD.cols()) - centersAUTD);
	autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(farPoints, amplitudes));
	autd.AppendModulation(autd::Modulation::Create(255));
}

void ocs::CreateFocusOnCenter(FloatingObjectPtr objPtr, Eigen::VectorXi const &amplitudes)
{
	Eigen::MatrixXf focus = centersAUTD + (objPtr->getPosition().replicate(1, centersAUTD.cols()) - centersAUTD);
	autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes));
	autd.AppendModulation(autd::Modulation::Create(255));
}

Eigen::VectorXf ocs::FindDutySI(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::VectorXf dutiesOffset(centersAUTD.cols()); dutiesOffset.setZero();
	Eigen::VectorXf gainPSi(centersAUTD.cols()); gainPSi.setConstant(-0.75); gainPSi[0] = -1.0;
	Eigen::VectorXf gainDSi(centersAUTD.cols()); gainDSi.setConstant(-1.3); gainDSi[0] = -1.8;
	Eigen::VectorXf gainISi(centersAUTD.cols()); gainISi.setConstant(-0.01);
	Eigen::VectorXf drRel = directionsAUTD.transpose() * dr;
	Eigen::VectorXf dvRel = directionsAUTD.transpose() * objPtr->getVelocity();
	Eigen::VectorXf diRel = directionsAUTD.transpose() * objPtr->getIntegral();
	Eigen::VectorXf duties = dutiesOffset + gainPSi.asDiagonal() * drRel + gainDSi.asDiagonal() * dvRel;// +gainI.asDiagonal() * diRel;
	return duties;
}

Eigen::VectorXf ocs::FindDutyQPEq(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dutiesOffset(0, 0, 0);
	Eigen::Vector3f gainPEq(-0.6, -0.6, -1.0);
	Eigen::Vector3f gainDEq(-1.0, -1.0, -1.8);
	Eigen::Vector3f gainIEq(0.0, 0.0, 0.0);
	Eigen::MatrixXf directions2obj = (objPtr->getPosition().replicate(1, centersAUTD.cols()) - centersAUTD).colwise().normalized();
	Eigen::VectorXf f = dutiesOffset + gainPEq.asDiagonal() * dr + gainDEq.asDiagonal() * objPtr->averageVelocity() + gainIEq.asDiagonal() * objPtr->getIntegral();
	Eigen::MatrixXf E = directions2obj.transpose() * directions2obj;
	Eigen::VectorXf b = -directions2obj.transpose() * f;
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Ed = dlib::mat(E);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(centersAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = 255 * dlib::ones_matrix<float>(centersAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centersAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Ed, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(centersAUTD.cols());
	for (int index = 0; index < centersAUTD.cols(); index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutySVD(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dv = objPtr->averageVelocity() - objPtr->getVelocityTarget();
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, centersAUTD.cols()) - centersAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::Vector3f gainPQp(-6e-3, -6e-3, -6e-3);
	Eigen::Vector3f gainDQp(-22e-3, -22e-3, -22e-3);
	Eigen::Vector3f gainIQp(-2e-4, -2e-4, -2e-4);
	Eigen::Vector3f force = gainPQp.asDiagonal() * dr
		+ gainDQp.asDiagonal() * dv
		+ gainIQp.asDiagonal() * objPtr->getIntegral()
		- 0.5 * F.rowwise().sum();
	Eigen::VectorXf duties =  F.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(force) + 0.5 * Eigen::VectorXf::Ones(F.cols());
	return duties;
}

Eigen::VectorXf ocs::FindDutyQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position, Eigen::VectorXf const &duty_forward)
{
	Eigen::MatrixXf posRel = position.replicate(1, centersAUTD.cols()) - centersAUTD;
	Eigen::MatrixXf F = arfModelPtr->arf(posRel, eulerAnglesAUTD);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty_reserved = Eigen::VectorXf::Ones(duty_forward.size()) - duty_forward;
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::mat(duty_reserved);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centersAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(centersAUTD.cols());
	for (int index = 0; index < NUM_AUTDS; index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutyQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position)
{
	return FindDutyQP(force, position, Eigen::VectorXf::Zero(positionsAUTD.cols()));
}

Eigen::VectorXf ocs::FindDutyMaximizeForce(Eigen::Vector3f const &direction,
	Eigen::MatrixXf const &constrainedDirections,
	Eigen::Vector3f const &position,
	Eigen::VectorXf const &duty_limit,
	float &force,
	Eigen::Vector3f const &force_offset)
{
	int const scale = 10000000; // variables are scaled for CGAL LP solver accepts only integer numbers
	Eigen::VectorXf result;
	Eigen::MatrixXf posRel = position.replicate(1, centersAUTD.cols()) - centersAUTD;
	force = EigenLinearProgrammingSolver(result,
		scale*constrainedDirections.transpose() * arfModelPtr->arf(posRel, eulerAnglesAUTD),
		-scale*constrainedDirections.transpose() * force_offset, //right hand side of constraints.
		-scale * direction.transpose() * arfModelPtr->arf(posRel, eulerAnglesAUTD), //formulation for minimization problem
		Eigen::VectorXi::Zero(constrainedDirections.cols()), //all the conditions are equality ones.
		Eigen::VectorXf::Zero(eulerAnglesAUTD.cols()), //lower bound
		scale * duty_limit,
		scale); //upper bound
	return result / scale;
}
