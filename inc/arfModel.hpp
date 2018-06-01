#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <Eigen\Dense>

namespace arfModel
{
	float arfX(float distance, float offset);

	float arfZ(float distance, float offset);

	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);

	Eigen::MatrixXf arfDirections(Eigen::MatrixXf posRel, Eigen::VectorXf duties, Eigen::MatrixXf directions);

	Eigen::Vector3f arfDirectionsTotal(Eigen::MatrixXf posRel, Eigen::VectorXf duties, Eigen::MatrixXf directions);

	Eigen::MatrixXf arfOffsets(const Eigen::MatrixXf posRel, const Eigen::VectorXf duties, const Eigen::MatrixXf offsets);

	Eigen::Vector3f arfTotalOffsets(const Eigen::MatrixXf posRel, const  Eigen::VectorXf duties, const Eigen::MatrixXf offsets);

	Eigen::MatrixXf arfOffsetsLocal(const Eigen::MatrixXf posRel, const Eigen::MatrixXf eulerAnglesAUTDS, const  Eigen::VectorXf duties, const Eigen::MatrixXf offsetsLocal);
	
	Eigen::Vector3f arfTotalOffsetsLocal(const Eigen::MatrixXf posRel, const Eigen::MatrixXf eulerAnglesAUTDS, const  Eigen::VectorXf duties, const Eigen::MatrixXf offsetsLocal);
}

#endif // !ARFMODEL_H_