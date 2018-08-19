#include <Eigen/Dense>

Eigen::VectorXf propagateLinearSystem(const Eigen::VectorXf &state,
	Eigen::VectorXf const &input,
	Eigen::MatrixXf const &A,
	Eigen::MatrixXf const &B)
{
	return A * state + B * input;
}

void estimateStateKF(Eigen::VectorXf &state,
	Eigen::MatrixXf &P,
	Eigen::VectorXf const &input,
	Eigen::VectorXf const &observation,
	Eigen::MatrixXf const &A,
	Eigen::MatrixXf const &B,
	Eigen::VectorXf const &c,
	Eigen::MatrixXf const &C,
	Eigen::MatrixXf const &D,
	Eigen::MatrixXf const &W,
	Eigen::MatrixXf const &V)
{
	Eigen::MatrixXf M = A*P*A.transpose() + D*W*D.transpose();
	Eigen::MatrixXf K = M * C.transpose()*(C*M*C + V);
	Eigen::VectorXf state_prop = A * state + B * input + c;
	Eigen::VectorXf e = observation - C * state_prop;
	P = (Eigen::MatrixXf::Identity(P.rows(), P.cols()) - K * C)*M;
	state = state_prop - K * e;
}
