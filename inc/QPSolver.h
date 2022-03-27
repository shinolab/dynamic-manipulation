#pragma once
#include <Eigen/Dense>

/// <summary>
/// Find x that minimizes c^T * x, subject to Ax <=/==/>= b
/// positive/zero/negative number of the equalityConditions[i] means Ax[i] <=/==/>= b[i]. 
/// </summary>
/// <param name="result"></param>
/// <param name="A"></param>
/// <param name="b"></param>
/// <param name="c"></param>
/// <param name="equalityConditions"></param>
/// <param name="lowerbound"></param>
/// <param name="upperbound"></param>
/// <param name="scale"></param>
/// <returns></returns>

float SolveLinearProgram(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound,
	float accuracy = 1.0e-8f
);

/*
Find x such that minimizes: 0.5*x'Dx + cx, subject to Ax >=/=/=< b, lb<=x<=ub
positive/zero/negative number of the equalityConditions[i] means Ax[i] <=/==/>= b[i]. 
*/
float SolveQuadraticProgram(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::MatrixXf const &D,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound,
	float accuracy = 1.0e-7f
);
