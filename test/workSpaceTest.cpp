#include "odcs.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>

int main() {
	ods sensor;
	sensor.Initialize();
	sensor.SetSensorGeometry(Eigen::Vector3f(-700, 0, 0), Eigen::Vector3f::Zero());	
	sensor.SetWorkSpace(Eigen::Vector3f(-315, -134, 1000), Eigen::Vector3f(0, 134, 2000));
	while (1) {
		sensor.GetPositionByDepth(FloatingObject::Create(Eigen::Vector3f(0, 0, 1500)), Eigen::Vector3f(), false);
		auto key = cv::waitKey(1);
		if (key == '27') { break; }
	}
	return 0;
}