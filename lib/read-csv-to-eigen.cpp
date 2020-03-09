#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "read-csv-to-eigen.hpp"

Eigen::MatrixXf readCSV(std::string file, int rows, int cols) {

  std::ifstream in(file);
  
  std::string line;

  int row = 0;
  int col = 0;

  Eigen::MatrixXf res(rows, cols);

  if (in.is_open()) {

    while (std::getline(in, line)) {

      char *ptr = (char *) line.c_str();
      int len = line.length();

      col = 0;

      char *start = ptr;
      for (int i = 0; i < len; i++) {

        if (ptr[i] == ',') {
          res(row, col++) = std::stof(start);
          start = ptr + i + 1;
        }
      }
      res(row, col) = std::stof(start);

      row++;
    }

    in.close();
  }
  return res;
}