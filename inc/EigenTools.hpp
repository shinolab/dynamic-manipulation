#ifndef _EIGEN_TOOLS_H
#define _EIGEN_TOOLS_H

#include <Eigen/Dense>

namespace Eigen {
	void FindTrue(Eigen::Array<bool, -1, -1> const &eval, Eigen::VectorXi &result, Eigen::VectorXi const &num) {
		result.resize(eval.rows());
		result.setConstant(-1);
		for (int iRow = 0; iRow < result.rows(); iRow++) {
			int count = -1;
			for (int iCol = 0; iCol < eval.cols(); iCol++) {
				if (eval(iRow, iCol)) { count++; }
				if (count == num(iRow)) {
					result(iRow) = iCol;
					break;
				}
			}
		}
	}
}

#endif