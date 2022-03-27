#pragma once
#include <Eigen/Dense>
#include <vector>

//Geometry Utility
std::vector<Eigen::Vector3f> cornersWorkspaceAll(Eigen::Vector3f const &corner0, Eigen::Vector3f const &corner1);
std::vector<float> range2Points(Eigen::Vector3f const& pos_self, Eigen::Quaternionf const &quo_self, std::vector<Eigen::Vector3f> const &points);
bool isInsideWorkspace(Eigen::Vector3f const& pos, Eigen::Vector3f const &lowerbound, Eigen::Vector3f const &upperbound);
