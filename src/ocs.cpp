#include "autd3.hpp"
#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <Windows.h>
#include <Eigen\Geometry>
#include <dlib\matrix.h>
#include <dlib\optimization.h>
//#include "engine.h"
#include "arfModel.hpp"
#include "additionalGain.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

#define NUM_AUTDS 5
#define NUM_STATES 6
#define NUM_NODES_DCNLP 11
//#define NUM_TABLE_OFFSET 21
//#define NUM_TABLE_DISTANCE 8

#pragma comment(lib, "winmm.lib")

int ocs::Initialize()
{
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) return ENXIO;

	/*
	Eigen::Vector3f positionAUTD0(415, 435, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-585, 435, 0); Eigen::Vector3f eulerAngleAUTD1(0, 0, 0);
	Eigen::Vector3f positionAUTD2(-585, -565, 0); Eigen::Vector3f eulerAngleAUTD2(0, 0, 0);
	Eigen::Vector3f positionAUTD3(415, -565, 0); Eigen::Vector3f eulerAngleAUTD3(0, 0, 0);
	*/
	Eigen::Vector3f positionAUTD0(-85, -65, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-1000, 65, 1000); Eigen::Vector3f eulerAngleAUTD1(M_PI, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD2(1000, -65, 1000); Eigen::Vector3f eulerAngleAUTD2(0, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD3(-65, -1000, 1000); Eigen::Vector3f eulerAngleAUTD3(-M_PI_2, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD4(65, 1000, 1000); Eigen::Vector3f eulerAngleAUTD4(M_PI_2, -M_PI_2, 0);

	autd.geometry()->AddDevice(positionAUTD0, eulerAngleAUTD0);
	autd.geometry()->AddDevice(positionAUTD1, eulerAngleAUTD1);
	autd.geometry()->AddDevice(positionAUTD2, eulerAngleAUTD2);
	autd.geometry()->AddDevice(positionAUTD3, eulerAngleAUTD3);
	autd.geometry()->AddDevice(positionAUTD4, eulerAngleAUTD4);

	positionAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	positionAUTD << positionAUTD0, positionAUTD1, positionAUTD2, positionAUTD3, positionAUTD4;
	eulerAnglesAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	eulerAnglesAUTD << eulerAngleAUTD0, eulerAngleAUTD1, eulerAngleAUTD2, eulerAngleAUTD3, eulerAngleAUTD4;
	centerAUTD.resize(3, NUM_AUTDS);//autd.geometry()->numDevices());
	directionsAUTD.resize(3, NUM_AUTDS);
	for (int i = 0; i < NUM_AUTDS; i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		Eigen::Affine3f affineG2AUTD = Eigen::Translation3f(positionAUTD.col(i)) * quo;
		centerAUTD.col(i) << affineG2AUTD * Eigen::Vector3f(85, 65, 0);
		directionsAUTD.col(i) << quo * Eigen::Vector3f::UnitZ();
	}
	/*
	if (!(ep = engOpen(""))) {
	fprintf(stderr, "\nCan't start MATLAB engine\n");
	return EXIT_FAILURE;
	}
	*/
	//engEvalString(ep, "cd 'C:\\Users\\takuro\\Documents\\MATLAB\\AcouticAviator'");
	return 0;
}

void ocs::Close()
{
	autd.Close();
	//engClose(ep);
}

//==================== Find Duty Module ====================

void ocs::FindDutyBruteForce(Eigen::VectorXf *const duties, Eigen::MatrixXf *const directions, const Eigen::Vector3f pos, const Eigen::Vector3f force)
{
	const int numPatternDuty = 10;
	const int numPatternDirection = 5;
	const int numPatternTotal = pow(numPatternDuty * numPatternDirection, NUM_AUTDS);
	float radius = 100;
	Eigen::MatrixXf posRel = pos.replicate(1, NUM_AUTDS) - centerAUTD;
	Eigen::Vector3f forceTemp = Eigen::Vector3f::Zero();
	for (int iPattern = 0; iPattern < numPatternTotal; iPattern++)
	{
		if (iPattern % 10000 == 0)
		{
			std::cout << iPattern << "th pattern " << std::endl;
		}
		Eigen::VectorXf dutiesTemp = Eigen::VectorXf::Zero(duties->rows());
		Eigen::MatrixXf directionsTemp = Eigen::MatrixXf::Zero(directions->rows(), directions->cols());
		for (int iAUTD = 0; iAUTD < NUM_AUTDS; iAUTD++)
		{
			const int numPatternDuties = std::pow(numPatternDuty, NUM_AUTDS);
			const int pDuty = std::pow(numPatternDuty, iAUTD);
			dutiesTemp[iAUTD] = (1.0 / numPatternDuty) * (iPattern % numPatternDuties % pDuty);//0 ~ numPatternDuty
			const int pDirection = std::pow(numPatternDirection, iAUTD);
			float angle = (2 * M_PI / numPatternDirection) * (iPattern % numPatternDuties % pDirection); // 0 ~ numPatternDirection
			if (angle = 0)
			{
				directionsTemp.col(iAUTD) << pos.normalized();
			}
			else
			{
				//UNIT_X VECTOR of AUTD i
				Eigen::Quaternionf quo =
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).x(), Eigen::Vector3f::UnitZ()) *
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).y(), Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf(eulerAnglesAUTD.col(iAUTD).z(), Eigen::Vector3f::UnitZ());
				Eigen::Vector3f unitXi = quo * Eigen::Vector3f::UnitX();
				//derive offset vector 
				directionsTemp.col(iAUTD) << pos + radius * unitXi.cross(pos).normalized();
			}
		}
		if ((force - arfModel::arfDirectionsTotal(posRel, dutiesTemp, directionsTemp)).norm() < (force - forceTemp).norm())
		{
			forceTemp = arfModel::arfDirectionsTotal(posRel, dutiesTemp, directionsTemp);
			*directions = directionsTemp;
			*duties = dutiesTemp;
		}
	}
}

//variable ; [u0 u1 ... uN xL0 xL1 ... xLN yL0 yL1 ... yLN] (offset should be represented in AUTD local coordinate)
double ocs::SqDiffOfForce(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	dataObj* d = reinterpret_cast<dataObj*>(data);
	Eigen::MatrixXf posRel = d->posRel;
	Eigen::Vector3f forceTarget = d->forceTarget;
	Eigen::MatrixXf eulerAnglesAUTD = d->eulerAngleAUTDS;
	std::vector<double> x_copy = x;
	Eigen::VectorXd duties = Eigen::Map<Eigen::VectorXd>(&x_copy[0], NUM_AUTDS);
	Eigen::MatrixXf offsetsL(3, NUM_AUTDS);
	offsetsL.row(0) = Eigen::Map<Eigen::RowVectorXd>(&x_copy[NUM_AUTDS], NUM_AUTDS).cast<float>();
	offsetsL.row(1) = Eigen::Map<Eigen::RowVectorXd>(&x_copy[2 * NUM_AUTDS], NUM_AUTDS).cast<float>();
	offsetsL.row(2).setZero();
	Eigen::MatrixXf offsetsG(3, NUM_AUTDS);
	for (int i = 0; i < NUM_AUTDS; i++)
	{
		Eigen::Quaternionf quo =
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).x(), Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).y(), Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(eulerAnglesAUTD.col(i).z(), Eigen::Vector3f::UnitZ());
		offsetsG.col(i) = quo.inverse() * offsetsL.col(i);
	}
	Eigen::Vector3f F = arfModel::arfTotalOffsets(posRel, duties.cast<float>(), offsetsG);
	return (F - forceTarget).norm();
}

//==================== Global Control Module ====================

void ocs::PositionControlBySingleAUTD(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f position = objPtr->getPosition();
	Eigen::Vector3f velocity = objPtr->averageVelocity();
	Eigen::Vector3f integral = objPtr->getIntegral();
	Eigen::Vector3f positionTarget = objPtr->getPositionTarget();
	Eigen::Vector3f velocityTaget = objPtr->getVelocityTarget();
	//corner of AUTD
	Eigen::Vector3f dr; dr << -positionTarget[0] + position[0], -positionTarget[1] + position[1], 0;
	Eigen::Vector3f drdt; drdt << velocity[0], velocity[1], 0;
	Eigen::Vector3f waveDirection; waveDirection = position + objPtr->radius * (dr + 0.5 * drdt).normalized() - centerAUTD;

	int amplitude = round(std::max(std::min(105 - 0.1 * (position[2] - positionTarget[2]) - 0.1 * velocity[2] - 0.001 * integral[2], 255.0), 0.0));
	if (objPtr->isTracked == true)
	{
		autd.AppendGainSync(autd::FocalPointGain::Create(100 * waveDirection));
		autd.AppendModulationSync(autd::Modulation::Create((uint8_t)amplitude));
		Sleep(25);
	}
	else
	{
		autd.AppendGainSync(autd::NullGain::Create());
	}
}

void ocs::DirectSemiPlaneWave(FloatingObjectPtr objPtr, Eigen::VectorXi amplitudes)
{
	float outpor = 100;
	Eigen::MatrixXf farPoints = centerAUTD + outpor * (objPtr->getPosition().replicate(1, centerAUTD.cols()) - centerAUTD);
	autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(farPoints, amplitudes));
	autd.AppendModulation(autd::Modulation::Create(255));
	Sleep(25);
}

//update the target position of the object so that the object follow the path
void ocs::followPath(FloatingObjectPtr objPtr)
{
	int latestNodeIndex = 0;
	int nextNodeIndex = 1;
	float tf = objPtr->timePath[objPtr->timePath.cols() - 1];
	float initialTime = (float)timeGetTime() / 1000.0;
	while (1)
	{
		float currentTime = (float)timeGetTime() / 1000.0 - initialTime;
		if (currentTime > objPtr->timePath[nextNodeIndex])
		{
			latestNodeIndex++;
			nextNodeIndex++;
			std::cout << "Passed : " << currentTime << std::endl;
			if (currentTime > tf)
			{
				break;
			}
		}
		else
		{
			//polynominal approximation of the path
			Eigen::VectorXf latestNodeState(NUM_STATES); latestNodeState = objPtr->statePath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeState(NUM_STATES); nextNodeState = objPtr->statePath.col(nextNodeIndex);
			Eigen::VectorXf latestNodeDerivative(NUM_STATES); latestNodeDerivative = objPtr->derivativePath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeDerivative(NUM_STATES); nextNodeDerivative = objPtr->derivativePath.col(nextNodeIndex);
			Eigen::VectorXf latestNodeControl(NUM_AUTDS); latestNodeControl = objPtr->controlPath.col(latestNodeIndex);
			Eigen::VectorXf nextNodeControl(NUM_AUTDS); nextNodeControl = objPtr->controlPath.col(nextNodeIndex);
			float dT = currentTime - objPtr->timePath[latestNodeIndex];
			float period = objPtr->timePath[nextNodeIndex] - objPtr->timePath[latestNodeIndex];
			float dS = dT / period;
			Eigen::Matrix<float, NUM_STATES, 4> adjNodeMat; adjNodeMat << nextNodeState, nextNodeDerivative*period, latestNodeState, latestNodeDerivative * period;
			Eigen::Matrix4f polyMat;
			polyMat <<
				-2, 3, 0, 0,
				1, -1, 0, 0,
				2, -3, 0, 1,
				1, -2, 1, 0;
			Eigen::Matrix<float, NUM_STATES, 4> polyCoefficients = adjNodeMat * polyMat;
			Eigen::Vector4f dSVec; dSVec << dS*dS*dS, dS*dS, dS, 1;
			Eigen::VectorXf stateTarget = polyCoefficients * dSVec;
			Eigen::VectorXf controlTarget = dS * nextNodeControl + (1 - dS) * latestNodeControl;
			objPtr->updateStatesTarget(stateTarget.block(0,0,0,2), stateTarget.block(0, 3, 0, 5));
		}
	}
}

//Legacy Module

Eigen::VectorXf ocs::findDutySI(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::VectorXf dutiesOffset(centerAUTD.cols()); dutiesOffset.setZero();
	Eigen::VectorXf gainPSi(centerAUTD.cols()); gainPSi.setConstant(-0.75); gainPSi[0] = -1.0;
	Eigen::VectorXf gainDSi(centerAUTD.cols()); gainDSi.setConstant(-1.3); gainDSi[0] = -1.8;
	Eigen::VectorXf gainISi(centerAUTD.cols()); gainISi.setConstant(-0.01);
	Eigen::VectorXf drRel = directionsAUTD.transpose() * dr;
	Eigen::VectorXf dvRel = directionsAUTD.transpose() * objPtr->getVelocity();
	Eigen::VectorXf diRel = directionsAUTD.transpose() * objPtr->getIntegral();
	Eigen::VectorXf duties = dutiesOffset + gainPSi.asDiagonal() * drRel + gainDSi.asDiagonal() * dvRel;// +gainI.asDiagonal() * diRel;
	return duties;
}

Eigen::VectorXf ocs::findDutyQPEq(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dutiesOffset(0, 0, 0);
	Eigen::Vector3f gainPEq(-0.6, -0.6, -1.0);
	Eigen::Vector3f gainDEq(-1.0, -1.0, -1.8);
	Eigen::Vector3f gainIEq(0.0, 0.0, 0.0);
	Eigen::MatrixXf directions2obj = (objPtr->getPosition().replicate(1, centerAUTD.cols()) - centerAUTD).colwise().normalized();
	Eigen::VectorXf f = dutiesOffset + gainPEq.asDiagonal() * dr + gainDEq.asDiagonal() * objPtr->averageVelocity() + gainIEq.asDiagonal() * objPtr->getIntegral();
	Eigen::MatrixXf E = directions2obj.transpose() * directions2obj;
	Eigen::VectorXf b = -directions2obj.transpose() * f;
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Ed = dlib::mat(E);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = 255 * dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Ed, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	Eigen::VectorXf duty(centerAUTD.cols());
	for (int index = 0; index < centerAUTD.cols(); index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutyQP(Eigen::Vector3f force, Eigen::Vector3f position)
{
	Eigen::MatrixXf posRel = position.replicate(1, centerAUTD.cols()) - centerAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty(NUM_AUTDS);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = 0.5 * dlib::ones_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	for (int index = 0; index < NUM_AUTDS; index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutyQP(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dv = objPtr->averageVelocity() - objPtr->getVelocityTarget();
	Eigen::Vector3f dutiesOffset(0, 0, 0);
	Eigen::Vector3f gainPQp(-3e-2, -3e-2, -3e-2);
	Eigen::Vector3f gainDQp(-11e-2, -11e-2, -11e-2);
	Eigen::Vector3f gainIQp(-5e-4, -5e-4, -5e-4);
	Eigen::Vector3f force = gainPQp.asDiagonal() * dr + gainDQp.asDiagonal() * dv + gainIQp.asDiagonal() * objPtr->getIntegral() - objPtr->gravityForce;
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, centerAUTD.cols()) - centerAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::MatrixXf Q = F.transpose() * F;
	Eigen::VectorXf b = -F.transpose() * force;
	Eigen::VectorXf duty(NUM_AUTDS);
	dlib::matrix<float, NUM_AUTDS, NUM_AUTDS> Qd = dlib::mat(Q);
	dlib::matrix<float, NUM_AUTDS, 1> bd = dlib::mat(b);
	dlib::matrix<float, NUM_AUTDS, 1> u = dlib::zeros_matrix<float>(NUM_AUTDS, 1);
	dlib::matrix<float, NUM_AUTDS, 1> upperbound = dlib::ones_matrix<float>(centerAUTD.cols(), 1);
	dlib::matrix<float, NUM_AUTDS, 1> lowerbound = dlib::zeros_matrix<float>(centerAUTD.cols(), 1);
	dlib::solve_qp_box_constrained(Qd, bd, u, lowerbound, upperbound, (float)1e-5, 100);
	for (int index = 0; index < NUM_AUTDS; index++)
	{
		duty[index] = u(index, 0);
	}
	return duty;
}

Eigen::VectorXf ocs::FindDutySVD(FloatingObjectPtr objPtr)
{
	Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dv = objPtr->averageVelocity() - objPtr->getVelocityTarget();
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, centerAUTD.cols()) - centerAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel);
	Eigen::Vector3f gainPQp(-6e-3, -6e-3, -6e-3);
	Eigen::Vector3f gainDQp(-22e-3, -22e-3, -22e-3);
	Eigen::Vector3f gainIQp(-2e-4, -2e-4, -2e-4);
	Eigen::Vector3f force = gainPQp.asDiagonal() * dr
		+ gainDQp.asDiagonal() * dv
		+ gainIQp.asDiagonal() * objPtr->getIntegral()
		- objPtr->gravityForce
		- 0.5 * F.rowwise().sum();
	Eigen::VectorXf duties =  F.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(force) + 0.5 * Eigen::VectorXf::Ones(F.cols());
	return duties;
}


/*
int ocs::SetPath(Eigen::Vector3f initialPosition, Eigen::Vector3f finalPosition, FloatingObject* obj)
{
if (ep == NULL)
{
std::cout << "ERROR : ep==NULL" << std::endl;
return EXIT_FAILURE;
}
double state0[NUM_STATES] = { initialPosition.x() / 1000, initialPosition.y() / 1000, initialPosition.z() / 1000, 0, 0, 0 };
double stateF[NUM_STATES] = { finalPosition.x() / 1000, finalPosition.y() / 1000, finalPosition.z() / 1000, 0, 0, 0 };
//2. state0 stateF‚Ìî•ñ‚ðmxArray‰»‚·‚é
mxArray *mxState0 = NULL, *mxStateF = NULL, *mxNumStates = NULL, *mxNumNodes = NULL, *mxPosAUTD = NULL;
mxState0 = mxCreateDoubleMatrix(6, 1, mxREAL); memcpy((void*)mxGetPr(mxState0), (void*)state0, sizeof(state0));
mxStateF = mxCreateDoubleMatrix(6, 1, mxREAL); memcpy((void*)mxGetPr(mxStateF), (void*)stateF, sizeof(stateF));
mxNumStates = mxCreateDoubleScalar(NUM_STATES);
mxNumNodes = mxCreateDoubleScalar(NUM_NODES_DCNLP);
std::cout << "centerAUTD\n" << centerAUTD << std::endl;
Eigen::MatrixXd cAd = centerAUTD.cast<double>() / 1000.0;
mxPosAUTD = mxCreateDoubleMatrix(3, NUM_AUTDS, mxREAL); memcpy((void*)mxGetPr(mxPosAUTD), (void*)cAd.data(), sizeof(double) * 3 * NUM_AUTDS);
//3. engPutVariable‚ð—˜—p‚µ‚ÄMATLAB‚Éî•ñ‚ð“n‚·
engPutVariable(ep, "state0", mxState0);
engPutVariable(ep, "stateF", mxStateF);
engPutVariable(ep, "numNodes", mxNumNodes);
engPutVariable(ep, "posAUTD", mxPosAUTD);
//4. destroyMxArray
mxDestroyArray(mxState0);
mxDestroyArray(mxStateF);
mxDestroyArray(mxNumNodes);
mxDestroyArray(mxPosAUTD);
//5. engEvalString‚ð—˜—p‚µ‚ÄMATLABã‚ÅŒo˜H¶¬
engEvalString(ep, "env = setEnvironment(state0, stateF, posAUTD, numNodes);");
//engEvalString(ep, "objfun = @(x) obj(x, env);");
//engEvalString(ep, "varOpt = fmincon(objfun, env.var0, env.Ain, env.bin, env.Aeq, env.beq, env.lb, env.ub, @(x) nonlcon(x, env), env.options);");
engEvalString(ep, "main2");
engEvalString(ep, "stateOpt = extractState(varOpt, env);");
engEvalString(ep, "controlOpt = extractControl(varOpt, env);");
engEvalString(ep, "tFOpt = extractTerminalTime(varOpt, env);");
engEvalString(ep, "tOpt = 0:tFOpt/(env.numNodes-1):tFOpt;");
engEvalString(ep, "derivativeOpt = stateEq(stateOpt, controlOpt, env);");
//=====Œo˜H‚ÌŽæ“¾======
//6. engGetVariable‚Å¶¬‚µ‚½Œo˜H‚ðŽæ“¾‚·‚éD
mxArray *mxStatePath = NULL, *mxDerivativePath = NULL, *mxControlPath = NULL, *mxTimePath = NULL;
mxStatePath = engGetVariable(ep, "stateOpt"); mxControlPath = engGetVariable(ep, "controlOpt");
mxTimePath = engGetVariable(ep, "tOpt"); mxDerivativePath = engGetVariable(ep, "derivativeOpt");
//7. result ‚ðEigen::Vector‚É•ÏŠ·‚·‚éD(result“à‚Ìdata‚ðMap‚·‚éD)
//array‚ðstd::vectorXf‚Åwrap
obj->statePath = Eigen::MatrixXd::Map((double*)mxGetData(mxStatePath), NUM_STATES, NUM_NODES_DCNLP).cast<float>();
obj->derivativePath = Eigen::MatrixXd::Map((double*)mxGetData(mxDerivativePath), NUM_STATES, NUM_NODES_DCNLP).cast<float>();
obj->controlPath = Eigen::MatrixXd::Map((double*)mxGetData(mxControlPath), NUM_AUTDS, NUM_NODES_DCNLP).cast<float>();
obj->timePath = Eigen::MatrixXd::Map((double*)mxGetData(mxTimePath), 1, NUM_NODES_DCNLP).cast<float>();
//8. destroyMxArray
obj->statePath *= 1000; obj->derivativePath *= 1000; //unit conversion (m -> mm)
mxDestroyArray(mxStatePath); mxDestroyArray(mxDerivativePath); mxDestroyArray(mxControlPath); mxDestroyArray(mxTimePath);
return EXIT_SUCCESS;
}
*/