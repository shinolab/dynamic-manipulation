#include <Eigen/Dense>
#include <iostream>

Eigen::VectorXf propagateLinearSystem(const Eigen::VectorXf &state,
	Eigen::VectorXf const &input,
	Eigen::VectorXf const &offset,
	Eigen::MatrixXf const &A,
	Eigen::MatrixXf const &B,
	Eigen::VectorXf const &constant)
{
	return A * state + B * input + constant + offset;
}

//estimate states of a discrete linear system x_next = A*x + B*input + g + D*w, y = C*x + v
void estimateStateKF(Eigen::VectorXf &state //state at the last step (this value will be updated.)
	, Eigen::MatrixXf &P // covariance matrix of the state at the last step (this value will be updated.)
	, Eigen::VectorXf const &input
	, Eigen::VectorXf const &observation
	, Eigen::MatrixXf const &A
	, Eigen::MatrixXf const &B
	, Eigen::VectorXf const &g
	, Eigen::MatrixXf const &C
	, Eigen::MatrixXf const &D
	, Eigen::MatrixXf const &W
	, Eigen::MatrixXf const &V)
{
	Eigen::MatrixXf M = A * P*A.transpose() + D * W*D.transpose();
	P << M - M * C.transpose()*(C*M*C.transpose() + V).inverse()*C*M;
	Eigen::MatrixXf K = P * C.transpose() * V.inverse();
	Eigen::VectorXf state_prop = A * state + B * input + g;
	Eigen::VectorXf e = observation - C * state_prop;
	state << state_prop + K * e;
}