#ifndef _ARFMODEL_H_
#define _ARFMODEL_H_

#include <Eigen\Dense>

class arfModelLinearBase
{
public:
	virtual Eigen::MatrixXf arf(Eigen::MatrixXf posRel) = 0;
	virtual Eigen::MatrixXf arf(Eigen::MatrixXf posRel, Eigen::MatrixXf directionsAUTD) = 0;
};

class arfModelConstant : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
};

class arfModelExperimentalPoly : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
};

class arfModelTheoreticalPoly : public arfModelLinearBase
{
public:
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
};

class arfModelExperimentalTable : public arfModelLinearBase
{
public:
	arfModelExperimentalTable();
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
private:
	Eigen::MatrixXf tableArfX;
	Eigen::MatrixXf tableArfZ;
	float arfX(float distance, float offset);
	float arfZ(float distance, float offset);
};

class arfModelTheoreticalTable 
{
public:
	arfModelTheoreticalTable();
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel);
	Eigen::MatrixXf arf(Eigen::MatrixXf posRel, Eigen::MatrixXf directionsAUTD);
private:
	Eigen::VectorXf tableDistance;
	Eigen::RowVectorXf tableAngle;
	Eigen::MatrixXf tableARF;
	
};


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