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

/*
Solve Linear Programming minimize: y = c'x subject to Ax </=/> b, .
sign of the coefficients of equality conditions specifies inequality/equlaity conditions of the row number.
*/
class LinearProgram {
public:
	struct Result {
		double residual;
		Eigen::VectorXf solution;
	};

	LinearProgram(
		const Eigen::MatrixXf& A,
		const Eigen::VectorXf& b,
		const Eigen::VectorXf& c,
		const Eigen::VectorXi& equalityConditions,
		const Eigen::VectorXf& lowerbound,
		const Eigen::VectorXf& upperbound,
		float accuracy = 1.0e-8f
	);

	Result solve();


private:
	Eigen::MatrixXf A;
	Eigen::VectorXf b, c, lowerbound, upperbound;
	Eigen::VectorXi equalityConditions;
	float accuracy;
};


/*
Find x such that minimizes: 0.5*x'Dx + cx, subject to Ax >=/=/=< b, lb<=x<=ub
positive/zero/negative number of the equalityConditions[i] means Ax[i] <=/==/>= b[i]. 
*/

class QuadraticProgram {
public:
	struct Result {
		double residual;
		Eigen::VectorXf solution;
	};

	QuadraticProgram(
		const Eigen::MatrixXf & A,
		const Eigen::VectorXf & b,
		const Eigen::MatrixXf & D,
		const Eigen::VectorXf & c,
		const Eigen::VectorXi & equalityConditions,
		const Eigen::VectorXf & lowerbound,
		const Eigen::VectorXf & upperbound,
		float accuracy = 1.0e-7f
	);

	Result solve();

private:
	Eigen::MatrixXf A, D;
	Eigen::VectorXf b, c, lowerbound, upperbound;
	Eigen::VectorXi equalityConditions;
	float accuracy;
};
