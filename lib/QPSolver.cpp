#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/QP_solution.h>
#include <Eigen/Dense>
#include <iostream>

// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

typedef CGAL::Linear_program_from_iterators
<const float**, const float*, CGAL::Comparison_result*, const bool*, const float*, const bool*, const float*, const float*> LpProgram;

typedef CGAL::Quadratic_program_from_iterators
<const float**, const float*, const CGAL::Comparison_result*, const bool*, const float*, const bool*, const float*, const float**, const float*> QpProgram;

typedef CGAL::Quadratic_program_from_iterators
<const int**, const int*, const CGAL::Comparison_result*, const bool*, const int*, const bool*, const int*, const int**, const int*> QpProgram_int;


typedef CGAL::Quadratic_program_solution<ET> Solution;

std::shared_ptr<const int*> Eigen2CgalArray2d(Eigen::MatrixXi const& eigenMat) {
	auto cgalMat = std::shared_ptr<const int*>(new const int* [eigenMat.cols()]);
	//auto cgalMat = std::make_shared<float*>(new float[eigenMat.cols()]);
	for (int i = 0; i < eigenMat.cols(); i++)
	{
		cgalMat.get()[i] = eigenMat.data() + i * eigenMat.rows();
	}
	return cgalMat;
}

std::shared_ptr<const float*> Eigen2CgalArray2d(Eigen::MatrixXf const &eigenMat) {
	auto cgalMat = std::shared_ptr<const float*>(new const float*[eigenMat.cols()]);
	//auto cgalMat = std::make_shared<float*>(new float[eigenMat.cols()]);
	for (int i = 0; i < eigenMat.cols(); i++)
	{
		cgalMat.get()[i] = eigenMat.data() + i * eigenMat.rows();
	}
	return cgalMat;
}

/*
Solve Linear Programming minimize: y = c'x subject to Ax </=/> b, .
sign of the coefficients of equality conditions specifies inequality/equlaity conditions of the row number.
*/
float EigenCgalLpSolver(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound,
	const float accuracy
)
{
	auto bound_scale_factor = 0.01f;

	Eigen::MatrixXf A_scaled = A / accuracy * bound_scale_factor;
	Eigen::VectorXf b_scaled = b / accuracy;
	Eigen::VectorXf c_scaled = c / accuracy;
	Eigen::VectorXf lb_scaled = lowerbound / bound_scale_factor;
	Eigen::VectorXf ub_scaled = upperbound / bound_scale_factor;
	auto _A_scaled = Eigen2CgalArray2d(A_scaled);

	CGAL::Comparison_result* r = new CGAL::Comparison_result[A.rows()];
	bool* flb = new bool[A.cols()];
	bool* fub = new bool[A.cols()];

	for (int i = 0; i < A.cols(); i++)
	{
		flb[i] = true;
		fub[i] = true;
	}

	for (int iCond = 0; iCond < A.rows(); iCond++)
	{
		if (equalityConditions[iCond] < 0)
			r[iCond] = CGAL::SMALLER;
		else if (equalityConditions[iCond] > 0)
			r[iCond] = CGAL::LARGER;
		else
			r[iCond] = CGAL::EQUAL;
	}
	LpProgram lp(A.cols(), A.rows(), _A_scaled.get(), b_scaled.data(), r, flb, lb_scaled.data(), fub, ub_scaled.data(), c_scaled.data(), 0.f);
	Solution s = CGAL::solve_linear_program(lp, ET());
	result.resize(A.cols());
	for (auto itr = s.variable_values_begin(); itr != s.variable_values_end(); itr++)
	{
		int index = std::distance(s.variable_values_begin(), itr);
		result[index] = (*itr).numerator().to_double() / (*itr).denominator().to_double();
	}
	result *= bound_scale_factor;
	delete fub;
	delete flb;
	delete r;
	return s.objective_value().numerator().to_double() / s.objective_value().denominator().to_double() * accuracy;
}

float EigenCgalQpSolver(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::MatrixXf const &D,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound,
	float accuracy
)
{
	Eigen::MatrixXf A_scaled = A / accuracy;
	Eigen::VectorXf b_scaled = b / accuracy;
	Eigen::MatrixXf D_scaled = D / accuracy;
	Eigen::VectorXf c_scaled = c / accuracy;
	auto _A_scaled = Eigen2CgalArray2d(A_scaled);
	auto _D_scaled = Eigen2CgalArray2d(D_scaled);
	CGAL::Comparison_result* r = new CGAL::Comparison_result[A.rows()];
	for (int i_eq = 0; i_eq < A.rows(); i_eq++) {
		if (equalityConditions[i_eq] < 0) {
			r[i_eq] = CGAL::SMALLER;
		}
		else if (equalityConditions[i_eq] > 0)
			r[i_eq] = CGAL::LARGER;
		else
			r[i_eq] = CGAL::EQUAL;
	}
	bool* flb = new bool[A.cols()];
	bool* fub = new bool[A.cols()];
	for (int i = 0; i < A.cols(); i++)
	{
		flb[i] = true;
		fub[i] = true;
	}

	QpProgram qp(A.cols(), A.rows(), _A_scaled.get(), b_scaled.data(), r, flb, lowerbound.data(), fub, upperbound.data(), _D_scaled.get(), c_scaled.data(), 0);
	Solution s = CGAL::solve_quadratic_program(qp, ET());
	result.resize(A.cols());
	for (auto itr = s.variable_values_begin(); itr != s.variable_values_end(); itr++)
	{
		int index = std::distance(s.variable_values_begin(), itr);
		result[index] = (*itr).numerator().to_double() / (*itr).denominator().to_double();
	}
	delete fub;
	delete flb;
	delete r;
	return s.objective_value().numerator().to_double() / s.objective_value().denominator().to_double() * accuracy;
}

//define QP

