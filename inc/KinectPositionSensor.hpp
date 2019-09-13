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
			Eigen::Vector3f const &corner0,
			Eigen::Vector3f const &corner1,
			bool useROI = true);
		bool updateStates(FloatingObjectPtr objPtr);
		Eigen::Quaternionf rotation();
		Eigen::Vector3f position();
	private:
		Eigen::Vector3f _pos;
		Eigen::Quaternionf _quo;
		Eigen::Vector3f _corner0;
		Eigen::Vector3f _corner1;
		KinectApp kinect;
		bool _useROI;
		std::vector<Eigen::Vector3f> cornersWorkspaceAll();
		void maskWorkspace(cv::Mat &mat);
		float rangeWorkspaceMin();
		float rangeWorkspaceMax();
		bool isInsideWorkspace(Eigen::Vector3f const &pos);
	};

}
