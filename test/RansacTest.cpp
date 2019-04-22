#include "odcs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

int main() {
	Eigen::Vector3f posLeft(-423.86f, 596.04f, 1331.f);

	odcs dynaman;
	dynaman.Initialize();
	dynaman.odsPtr->SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1570.f));
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999986, -0.000739038, 0.00521048,
		0.00518096, 0.0355078, 0.999356,
		-0.000923425, 0.999369, -0.0355035;
	dynaman.odsPtr->SetSensorGeometry(Eigen::Vector3f(35.1867f, -1242.32f, 1085.62f), rotationKinect2Global);

	FloatingObjectPtr objPtr = FloatingObject::Create(posLeft, -0.0001f);

	while (1) {
		auto k = dynaman.Sensor()->kinect();
		cv::Mat depthImageRaw(k->getDepthHeight(), k->getDepthWidth(), CV_16UC1, &(k->depthBuffer[0]));
		cv::Mat depthImageUc8;
		depthImageRaw.convertTo(depthImageUc8, CV_8UC1, 255.0 / (float)k->depthMaxReliableDistance, 0);
		cv::Mat img;
		cv::cvtColor(depthImageUc8, img, cv::COLOR_GRAY2BGR);
		Eigen::Vector3f position;
		dynaman.Sensor()->GetPoisitionRansac(objPtr, position, true);
		if (true) {
			CameraSpacePoint csp = { position.x(), position.y(), position.z() };
			DepthSpacePoint dsp;
			k->coordinateMapper->MapCameraPointToDepthSpace(csp, &dsp);
			cv::circle(img, cv::Point(dsp.X, dsp.Y), 3, cv::Scalar(0, 0, 255), -1);
		};
		auto key = cv::waitKey(500);
		if (key == 'q') {
			break;
		}
		cv::imshow("RANSAC", img);
	}
	dynaman.Close();
}