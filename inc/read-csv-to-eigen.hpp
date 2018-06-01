#ifndef _READ_CSV_TO_EIGEN_H_
#define _READ_CSV_TO_EIGEN_H_
#include <Eigen\Dense>

Eigen::MatrixXf readCSV(std::string file, int rows, int cols);

#endif