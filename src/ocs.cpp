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

#define NUM_AUTDS 12

int ocs::Initialize()
{
	_autd.Open(autd::LinkType::ETHERCAT);
	if (!_autd.isOpen()) return ENXIO;

	//arfModelPtr.reset(new arfModelTheoreticalTable());
	arfModelPtr.reset(new arfModelFocusOnSphereExperimental());
	return 0;
}

void ocs::Close()
{
	_autd.Close();
	//engClose(ep);
}

int ocs::AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles) {
	Eigen::MatrixXf positionsAutdNew(3, _autd.geometry()->numDevices() + 1);
	positionsAutdNew << positionsAUTD, position;
	positionsAUTD = positionsAutdNew;
	Eigen::MatrixXf eulerAnglesAutdNew(3, _autd.geometry()->numDevices() + 1);
	eulerAnglesAutdNew << eulerAnglesAUTD, eulerAngles;
	eulerAnglesAUTD = eulerAnglesAutdNew;
	return _autd.geometry()->AddDevice(position, eulerAngles);
}

Eigen::MatrixXf ocs::CentersAUTD() {
	Eigen::MatrixXf centersAUTD(3, _autd.geometry()->numDevices());
	for (int i = 0; i < _autd.geometry()->numDevices(); i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		Eigen::Affine3f affineGlobal2AUTD = Eigen::Translation3f(positionsAUTD.col(i)) * quo;
		centersAUTD.col(i) << affineGlobal2AUTD * Eigen::Vector3f(86.36f, 66.04f, 0);
	}
	return centersAUTD;
}

Eigen::MatrixXf ocs::DirectionsAUTD() {
	Eigen::MatrixXf directionsAUTD(3, _autd.geometry()->numDevices());
	for (int i = 0; i < _autd.geometry()->numDevices(); i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		directionsAUTD.col(i) << quo * Eigen::Vector3f::UnitZ();
	}
	return directionsAUTD;
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

autd::GainPtr ocs::CreateGain(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f accel
		= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
		+ gainD.asDiagonal() * (objPtr->averageVelocity() - objPtr->getVelocityTarget())
		+ gainI.asDiagonal() * objPtr->getIntegral()
		+ objPtr->getAccelTarget();
	Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
	Eigen::VectorXf duties = FindDutySelectiveQP(forceToApply, objPtr->getPosition(), 0.5);
	Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
	Eigen::MatrixXf focus = CentersAUTD() + (objPtr->getPosition().replicate(1, CentersAUTD().cols()) - CentersAUTD());
	return autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes);
}

Eigen::VectorXf ocs::FindDutySI(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::VectorXf dutiesOffset(CentersAUTD().cols()); dutiesOffset.setZero();
	Eigen::VectorXf gainPSi(CentersAUTD().cols()); gainPSi.setConstant(-0.75); gainPSi[0] = -1.0;
	Eigen::VectorXf gainDSi(CentersAUTD().cols()); gainDSi.setConstant(-1.3); gainDSi[0] = -1.8;
	Eigen::VectorXf gainISi(CentersAUTD().cols()); gainISi.setConstant(-0.01);
	Eigen::VectorXf drRel = DirectionsAUTD().transpose() * dr;
	Eigen::VectorXf dvRel = DirectionsAUTD().transpose() * objPtr->getVelocity();
	Eigen::VectorXf diRel = DirectionsAUTD().transpose() * objPtr->getIntegral();
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
	Eigen::MatrixXf directions2obj = (objPtr->getPosition().replicate(1, CentersAUTD().cols()) - CentersAUTD()).colwise().normalized();
	Eigen::VectorXf f = dutiesOffset + gainPEq.asDiagonal() * dr + gainDEq.asDiagonal() * objPtr->averageVelocity() + gainIEq.asDiagonal() * objPtr->getIntegral();
	Eigen::MatrixXf E = directions2obj.transpose() * directions2obj;
	Eigen::VectorXf b = -directions2obj.transpose() * f;
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Ed = dlib::mat(E);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(CentersAUTD().cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = 255 * dlib::ones_matrix<float>(CentersAUTD().cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(CentersAUTD().cols(), 1);
	dlib::solve_qp_box_constrained(Ed, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(CentersAUTD().cols());
	for (int index = 0; index < CentersAUTD().cols(); index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutySVD(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dv = objPtr->averageVelocity() - objPtr->getVelocityTarget();
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, CentersAUTD().cols()) - CentersAUTD();
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
	Eigen::MatrixXf posRel = position.replicate(1, CentersAUTD().cols()) - CentersAUTD();
	Eigen::MatrixXf F = arfModelPtr->arf(posRel, eulerAnglesAUTD);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty_reserved = Eigen::VectorXf::Ones(duty_forward.size()) - duty_forward;
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::mat(duty_reserved);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(CentersAUTD().cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(CentersAUTD().cols());
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

Eigen::VectorXf ocs::FindDutyQPCGAL(Eigen::Vector3f const &force, Eigen::Vector3f const &position) {
	Eigen::VectorXf result;
	Eigen::MatrixXf posRel = position.replicate(1, _autd.geometry()->numDevices()) - CentersAUTD();
	Eigen::MatrixXf F = arfModelPtr->arf(posRel, eulerAnglesAUTD);
	EigenCgalQpSolver(result,
		Eigen::MatrixXf::Identity(_autd.geometry()->numDevices(), _autd.geometry()->numDevices()),
		Eigen::VectorXf::Zero(_autd.geometry()->numDevices()),
		F.transpose() * F,
		-F.transpose() * force,
		Eigen::VectorXi::Ones(_autd.geometry()->numDevices()),
		Eigen::VectorXf::Zero(_autd.geometry()->numDevices()),
		Eigen::VectorXf::Ones(_autd.geometry()->numDevices())
	);
	return std::move(result);
}

Eigen::VectorXf ocs::FindDutySelectiveQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position, float const threshold) {
	Eigen::MatrixXf posRel = (position.replicate(1, _autd.geometry()->numDevices()) - CentersAUTD());
	//Choose effective autds based on their radiation angle to avoid modelling errors and undesirable reflections.
	Eigen::ArrayXf innerProducts = (posRel.colwise().normalized().array() * DirectionsAUTD().array()).colwise().sum();
	Eigen::Array<bool, 1, Eigen::Dynamic> isEffective = (innerProducts >= threshold) && (innerProducts < 0.97);
	Eigen::MatrixXf selector = Eigen::MatrixXf::Zero(isEffective.size(), isEffective.count());
	for (int iRow = 0, iCol = 0; iCol < selector.cols() && iRow < selector.rows();) {
		if (isEffective(iRow)) {
			selector(iRow, iCol) = 1.0f;
			iCol++;
		}
		iRow++;
	}
	Eigen::MatrixXf F = arfModelPtr->arf(posRel * selector, eulerAnglesAUTD * selector);
	Eigen::VectorXf result_reduced;
	EigenCgalQpSolver(result_reduced,
		Eigen::MatrixXf::Identity(isEffective.count(), isEffective.count()),
		Eigen::VectorXf::Zero(isEffective.count()),
		F.transpose() * F,
		-F.transpose() * force,
		Eigen::VectorXi::Ones(isEffective.count()),
		Eigen::VectorXf::Zero(isEffective.count()),
		Eigen::VectorXf::Ones(isEffective.count())
	);
	return selector * result_reduced;	//expanded to specify which autd to use.

}

Eigen::VectorXf ocs::FindDutyMaximizeForce(Eigen::Vector3f const &direction,
	Eigen::MatrixXf const &constrainedDirections,
	Eigen::Vector3f const &position,
	Eigen::VectorXf const &duty_limit,
	float &force,
	Eigen::Vector3f const &force_offset)
{
	Eigen::VectorXf result;
	Eigen::MatrixXf posRel = position.replicate(1, CentersAUTD().cols()) - CentersAUTD();
	force = EigenLinearProgrammingSolver(result,
		constrainedDirections.transpose() * arfModelPtr->arf(posRel, eulerAnglesAUTD),
		-constrainedDirections.transpose() * force_offset, //right hand side of constraints.
		-direction.transpose() * arfModelPtr->arf(posRel, eulerAnglesAUTD), //formulation for minimization problem
		Eigen::VectorXi::Zero(constrainedDirections.cols()), //all the conditions are equality ones.
		Eigen::VectorXf::Zero(eulerAnglesAUTD.cols()), //lower bound
		duty_limit,
		1.0e-8f); //upper bound
	return result;
}
