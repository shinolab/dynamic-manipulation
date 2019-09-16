#ifndef _KINECT_DEPTH_SENSOR_HPP
#define _KINECT_DEPTH_SENSOR_HPP

#include "odcs.hpp"
#include "KinectApp.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <vector>

namespace dynaman {

	class KinectDepthPositionSensor : public Sensor {
	public:
		KinectDepthPositionSensor(Eigen::Vector3f const &pos,
			Eigen::Quaternionf const &quo,
			bool useROI = true);
		bool updateStates(FloatingObjectPtr objPtr);
		Eigen::Quaternionf rotation();
		Eigen::Vector3f position();
	private:
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		bool _useROI;
		KinectApp kinect;
		void maskWorkspace(Eigen::Vector3f const &lowerbound, Eigen::Vector3f const & upperbound, cv::Mat& mask);
	};

}


#endif // !_KINECT_DEPTH_SENSOR_HPP


