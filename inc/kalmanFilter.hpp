#ifndef _KALMAN_FILTER_HPP_
#define _KALMAN_FILTER_HPP_

#include <Eigen/Dense>

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
	Eigen::MatrixXf const &V
);

#endif