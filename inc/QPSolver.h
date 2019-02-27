#ifndef _ODCS_QP_SOLVER_H_
#define _ODCS_QP_SOLVER_H_

#include <Eigen/Dense>

float EigenLinearProgrammingSolver(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound,
	int const scale = 10000000
);

#endif