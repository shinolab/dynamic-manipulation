#include "odcs.hpp"
#include "autd3.hpp"
#include "arfModel.hpp"
#include "additionalGain.hpp"
#include "QPSolver.h"
#include <Eigen\Geometry>
#include <boost/math/common_factor_rt.hpp>
#include <algorithm>
#include <vector>
#include <deque>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace dynaman;

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
	Eigen::MatrixXf eulerAnglesAutdNew(3, _autd.geometry()->numDevices() + 1);
	if (_autd.geometry()->numDevices() == 0) {
		positionsAutdNew << position;
		eulerAnglesAutdNew << eulerAngles;
	}
	else {
		positionsAutdNew << positionsAUTD, position;
		eulerAnglesAutdNew << eulerAnglesAUTD, eulerAngles;
	}
	positionsAUTD = positionsAutdNew;
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

autd::GainPtr ocs::CreateBalanceGain(FloatingObjectPtr objPtr, int numObj)
{
	Eigen::Vector3f accel
		= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
		+ gainD.asDiagonal() * (objPtr->averageVelocity() - objPtr->getVelocityTarget())
		+ gainI.asDiagonal() * objPtr->getIntegral()
		+ objPtr->getAccelTarget();
	Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
	Eigen::VectorXf duties = numObj * FindDutySelectiveQP(forceToApply, objPtr->getPosition(), 0.5);
	Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
	Eigen::MatrixXf focus = CentersAUTD() + (objPtr->getPosition().replicate(1, CentersAUTD().cols()) - CentersAUTD());
	return autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes);
}

std::vector<autd::GainPtr> ocs::CreateBalanceGainMulti(std::vector<FloatingObjectPtr> const &objPtrs) {
	std::vector<autd::GainPtr> gain_list;
	
	int num_autd = _autd.geometry()->numDevices();
	int num_object = objPtrs.size();
	Eigen::Matrix3Xf positions(3, num_object); Eigen::Matrix3Xf positionsTarget(3, num_object);
	Eigen::Matrix3Xf velocities(3, num_object); Eigen::Matrix3Xf velocitiesTarget(3, num_object);
	Eigen::Matrix3Xf integrals(3, num_object);
	Eigen::Matrix3Xf accelsTarget(3, num_object);
	Eigen::Matrix3Xf forcesToApply(3, num_object);
	std::cout << "calcurating force..." << std::endl;
	for (auto itrObj = objPtrs.begin(); itrObj != objPtrs.end(); itrObj++) {
		int index = std::distance(objPtrs.begin(), itrObj);
		positions.col(index) = (*itrObj)->getPosition();
		Eigen::Vector3f accel
			= gainP.asDiagonal() * ((*itrObj)->getPosition() - (*itrObj)->getPositionTarget())
			+ gainD.asDiagonal() * ((*itrObj)->averageVelocity() - (*itrObj)->getVelocityTarget())
			+ gainI.asDiagonal() * (*itrObj)->getIntegral()
			+ (*itrObj)->getAccelTarget();
		forcesToApply.col(index) = (*itrObj)->totalMass() * accel + (*itrObj)->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
	}
	std::cout << "programming duties..." << std::endl;
	const Eigen::VectorXf duties = FindDutyQPMulti(forcesToApply, positions);
	std::cout << "constructing duties matrix..." << std::endl;
	const Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> duties_mat(duties.data(), num_object, num_autd);
	Eigen::Array<bool, -1, -1> nonzero = duties_mat.array().abs() > 1.0e-3f;
	Eigen::RowVectorXi count_nonzero = nonzero.matrix().cast<int>().colwise().sum();

	//Determine the number of gains:
	int num_gains = 1;
	for (int i_autd = 0; i_autd < num_autd; i_autd++) {
		if (count_nonzero[i_autd] != 0) {
			num_gains = boost::math::lcm(num_gains, count_nonzero[i_autd]);
		}
	}
	std::vector<int> counter(num_autd, -1);

	for (int i_gain = 0; i_gain < num_gains; i_gain++) {
		std::map<int, autd::GainPtr> gain_map;
		for (auto itr_counter = counter.begin(); itr_counter != counter.end(); itr_counter++) {
			autd::GainPtr gain;
			int i_autd = std::distance(counter.begin(), itr_counter);

			for (int i_obj = 0; i_obj < num_object; i_obj++) {
				(*itr_counter)++;
				if ((*itr_counter) == num_object) {
					*itr_counter = 0;
				}
				if (nonzero(*itr_counter, i_autd)) {
					int amplitude = 255 * std::sqrt(std::max(std::min(count_nonzero(i_autd) * duties_mat(*itr_counter, i_autd), 1.0f), 0.0f));
					gain = autd::FocalPointGain::Create(objPtrs[*itr_counter]->getPosition(), amplitude);
					//Create FocalPointGain
				}
				if (i_obj == num_object - 1) { // in case that the autd is inactive.
					gain = autd::NullGain::Create();
					//Create Null Gain
				}
			}
			gain_map.insert(std::make_pair(std::distance(counter.begin(), itr_counter), gain));
		}
		gain_list.push_back(autd::GroupedGain::Create(gain_map));
	}
	return gain_list;	
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

Eigen::VectorXf ocs::FindDutyQPMulti(Eigen::Matrix3Xf const &forces, Eigen::Matrix3Xf const &positions, float const penalty) {
	Eigen::VectorXf result;
	auto numObjects = positions.cols();
	auto numDevices = _autd.geometry()->numDevices();
	int dimResult = numObjects * numDevices;
	Eigen::MatrixXf A(numDevices, dimResult);
	Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3 * numObjects, dimResult);
	for (int i = 0; i < numObjects; i++) {
		Eigen::MatrixXf posRel = positions.col(i).replicate(1, numDevices) - CentersAUTD();
		F.block(3*i, i*numDevices, 3, numDevices) = arfModelPtr->arf(posRel, eulerAnglesAUTD);
		A.block(0, i*numDevices, numDevices, numDevices) = Eigen::MatrixXf::Identity(numDevices, numDevices);
	}
	Eigen::Map<const Eigen::VectorXf> fTgt(forces.data(), forces.size());
	EigenCgalQpSolver(result,
		A, //ieq. cond
		Eigen::VectorXf::Ones(numDevices), //sum of duties for each device must be smaller than one.
		F.transpose()*F,
		-F.transpose()*fTgt,
		-Eigen::VectorXi::Ones(numDevices), //equality conditions
		Eigen::VectorXf::Zero(dimResult), //lower bound
		Eigen::VectorXf::Ones(dimResult) //upper bound
	);
	return std::move(result);
}

