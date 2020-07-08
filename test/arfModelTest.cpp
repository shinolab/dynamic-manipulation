#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include "arfModel.hpp"

int main(int argc, char** argv) {
	Eigen::VectorXf rangeCol(4);
	Eigen::VectorXf rangeRow(3);
	rangeCol << 0, 1, 2, 3;
	rangeRow << 100, 400, 600;
	Eigen::MatrixXf table(rangeRow.size(), rangeCol.size());
	table << 1, 2, 3, 4,
		10, 20, 30, 40,
		100, 200, 300, 400;
	float valCol = 2.5;
	float valRow = 600;
	auto indexCol = SearchIntervalIndex(valCol, rangeCol);
	auto indexRow = SearchIntervalIndex(valRow, rangeRow);
	std::cout << "RangeRow: " << rangeRow.transpose() << std::endl;
	std::cout << "RangeCol: " << rangeCol.transpose() << std::endl;
	std::cout << "RangeRow:[" << indexRow << "] includes a value of " << valRow << std::endl;
	std::cout << "RangeCol:[" << indexCol << "] includes a value of " << valCol << std::endl;

	std::cout << "table \n" << table << std::endl;
	std::cout << "func(" << valCol << ", " << valRow << ") is estimated to: " << linearInterp2d(valRow, valCol, rangeRow, rangeCol, table) << std::endl;
	return 0;

}