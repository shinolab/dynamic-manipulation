#include "odcs.hpp"
#include "projector.hpp"
#include "profile.hpp"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>
#include <fstream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

int main()
{
	//initialization
	odcs odcs;
	odcs.Initialize();
	Eigen::Affine3f affineKinect2Global = odcs.odsPtr->getAffineKinect2Global();
	Eigen::Affine3f affineGlobal2Kinect = affineKinect2Global.inverse();
	Eigen::Matrix3f dcmGlobal2Kinect = odcs.odsPtr->getDcmGlobal2Kinect();
	odcs.AddObject(Eigen::Vector3f(0, 0, 1300));
	//start control thread.
	odcs.StartControl();

	//start projection thread.
	std::thread threadProjection([&odcs, &affineGlobal2Kinect, &dcmGlobal2Kinect](){
		cv::VideoCapture cap("video/bird2.mp4");
		cv::Mat phone0 = cv::imread("img/phone0b.png");
		cv::Mat phone1 = cv::imread("img/phone1b.png");
		cv::Mat phone2 = cv::imread("img/phone2b.png");
		cv::Mat phone3 = cv::imread("img/phone3b.png");

		std::string projectorName = "projector1";
		projector proj(projectorName);
		const int num_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
		int count_frame = 0;
		int num_average = 3;
		int period = 100;
		Eigen::MatrixXf posBuffer(3, num_average);
		unsigned int colcount = 0;
		while (1)
		{
			DWORD initTimeLoop = timeGetTime();
			if (odcs.GetFloatingObject(0)->isTracked)
			{
				Eigen::Vector3f pos = affineGlobal2Kinect * (odcs.GetFloatingObject(0)->getPosition());
				posBuffer.col(colcount%num_average) << pos;
				//Eigen::Vector3f vel = dcmGlobal2Kinect * odcs.GetFloatingObject(0)->getVelocity();
				colcount++;
				cv::Size earthSize(193, 198);
				//cap >> image;
				//projectImageOnObject("FULL", pos + vel * (timeGetTime() - odcs.GetFloatingObject(0)->lastDeterminationTime)/1000, image);
				//proj.projectImageOnObject(posBuffer.rowwise().sum() / num_average, image, cv::Size(187, 89), cv::Scalar::all(0)); // for VR LOGO

				
				auto key = cv::waitKey(1);
				count_frame++;
				/*
				count_frame++;
				if (count_frame >= num_frame / 2)
				{
				cap.set(cv::CAP_PROP_POS_FRAMES, 0);
				count_frame = 0;
				}
				*/
				if (key == 'q') { break; }
				int loopTime = (timeGetTime() - initTimeLoop);
				if (loopTime < 30) { Sleep(30 - loopTime); }
			}
		}
	});
	/*
	
	*/

	std::ofstream ofs("20181129_xy1300retry2.csv");
	float lengthX = 500;
	float offsetX = 0;
	float offsetY = 0;
	float offsetZ = 1300;
	int period = 20000;
	//Sleep(20000);
	ofs << "time, x, y, z, x_tgt, y_tgt, z_tgt, v_x, v_y, v_z, u0, u1, u2, u3, u4" << std::endl;
	DWORD initialTime = timeGetTime();
	DWORD loopPeriod = 30;
	while ((timeGetTime()-initialTime) <  period * 20)
	{
		DWORD beginningOfLoop = timeGetTime();
		float phase = 2 * M_PI * ((timeGetTime() - initialTime) % period) / (float)period;
		float omega = 2 * M_PI * 1000 / period;
		DWORD t = beginningOfLoop - initialTime;
		float x, y, z, vx, vy, vz;
		vx = 0; vy = 0; vz = 0;
		x = 35 * (t / period); y = 35 * (t / period); z = offsetZ;

		
		odcs.GetFloatingObject(0)->updateStatesTarget(Eigen::Vector3f(x, y, z), Eigen::Vector3f(vx, vy, vz));
		Eigen::Vector3f currentPosition = odcs.GetFloatingObject(0)->getPosition();
		ofs << timeGetTime() - initialTime << ", " << currentPosition.x() << ", " << currentPosition.y() << ", " << currentPosition.z() << ", "
			<< x << ", " << y << "," << z << ", " << vx << ", " << vy << ", " << vz << std::endl;
		int loopTime = timeGetTime() - beginningOfLoop;
		if (loopTime < loopPeriod)
		{
			Sleep(loopPeriod - loopTime);
		}
	}
	odcs.GetFloatingObject(0)->updateStatesTarget(Eigen::Vector3f(offsetX, offsetY, offsetZ-100), Eigen::Vector3f(0, 0, 0));
	std::cout << "transition to interaction mode." << std::endl;
	
	//start interaction thread.
	//interaction sequence
	/*
	KinectApp app;
	app.initialize();
	bool isGrabbed = false;
	DWORD timeGrabbed;
	const float distThreshold = 500;
	const int timeThreshold = 1000;
	std::cout << "initialization completed." << std::endl;
	initialTime = timeGetTime();
	while (timeGetTime() - initialTime < 90000)
	{
		DWORD beginningOfLoop = timeGetTime();
		HRESULT hr = app.getBodies();
		Eigen::Vector3f objectPosition = odcs.GetFloatingObject(0)->getPosition();
		Eigen::Vector3f targetPosition = odcs.GetFloatingObject(0)->getPositionTarget();

		if (SUCCEEDED(hr))
		{
			for (auto itr = app.pBodies.begin(); itr != app.pBodies.end(); itr++)
			{
				BOOLEAN isTracked = false;
				HRESULT hrTrack = (*itr)->get_IsTracked(&isTracked);
				if (isTracked && SUCCEEDED(hrTrack))
				{
					std::vector<Joint> joints(JointType::JointType_Count);
					HRESULT hrJoints = (*itr)->GetJoints(JointType::JointType_Count, &joints[0]);
					if (SUCCEEDED(hrJoints))
					{
						CameraSpacePoint cspHandRight = joints[JointType::JointType_HandRight].Position;
						CameraSpacePoint cspHandLeft = joints[JointType::JointType_HandLeft].Position;
						DepthSpacePoint dspHandRight = app.convertPositionToDepthPixel(cspHandRight);
						DepthSpacePoint dspHandLeft = app.convertPositionToDepthPixel(cspHandLeft);
						Eigen::Vector3f positionHandRight =
							affineKinect2Global * Eigen::Vector3f(1000 * cspHandRight.X, 1000 * cspHandRight.Y, 1000 * cspHandRight.Z);
						Eigen::Vector3f positionHandLeft =
							affineKinect2Global * Eigen::Vector3f(1000 * cspHandLeft.X, 1000 * cspHandLeft.Y, 1000 * cspHandLeft.Z);
						float distR = (objectPosition - positionHandRight).norm();
						float distL = (objectPosition - positionHandLeft).norm();
						cv::Mat depthImage(app.getDepthHeight(), app.getDepthWidth(), CV_16UC1);
						HRESULT hrDepth = app.getDepthBuffer();
						depthImage = cv::Mat(app.getDepthHeight(), app.getDepthWidth(), CV_16UC1, &app.depthBuffer[0]);
						cv::Mat grayView(app.getDepthHeight(), app.getDepthWidth(), CV_8UC1);
						depthImage.convertTo(grayView, CV_8UC1, 255.0 / (float)app.depthMaxReliableDistance, 0);
						cv::Mat view(app.getDepthHeight(), app.getDepthWidth(), CV_8UC3);
						cv::cvtColor(grayView, view, CV_GRAY2BGR);
						cv::circle(view, cv::Point(dspHandRight.X, dspHandRight.Y), 10, cv::Scalar(0, 0, 255));
						cv::circle(view, cv::Point(dspHandLeft.X, dspHandLeft.Y), 10, cv::Scalar(0, 255, 0));
						cv::putText(view, std::to_string(distR), cv::Point(0, 50), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255));
						cv::putText(view, std::to_string(distL), cv::Point(0, 100), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0));

						if (distR < distThreshold || distL < distThreshold)
						{
							if (isGrabbed)
							{
								if ((timeGetTime() - timeGrabbed) > timeThreshold)
								{
									if (distR < distThreshold)
									{
										odcs.GetFloatingObject(0)->updateStatesTarget(odcs.GetFloatingObject(0)->getPosition(), Eigen::Vector3f(0, 0, 0));
										cv::putText(view,
											"Following (R)",
											cv::Point(dspHandRight.X, dspHandRight.Y),
											CV_FONT_HERSHEY_SIMPLEX,
											0.6,
											cv::Scalar(0, 0, 255)
										);
									}
									else
									{
										odcs.GetFloatingObject(0)->updateStatesTarget(odcs.GetFloatingObject(0)->getPosition(), Eigen::Vector3f(0, 0, 0));
										cv::putText(view,
											"Following(L)",
											cv::Point(dspHandLeft.X, dspHandLeft.Y),
											CV_FONT_HERSHEY_SIMPLEX,
											0.6,
											cv::Scalar(0, 255, 0)
										);
									}
								}
							}
							else
							{
								isGrabbed = true;
								timeGrabbed = timeGetTime();
							}
						}
						else
						{
							isGrabbed = false;
						}

						if (SUCCEEDED(hrDepth))
						{
							cv::imshow("VIEW", view);
						}
					}
				}
			}
		}
		ofs << timeGetTime() - initialTime
			<< ", " << objectPosition.x() << ", " << objectPosition.y() << ", " << objectPosition.z()
			<< ", " << targetPosition.x() << ", " << targetPosition.y() << ", " << targetPosition.z() << std::endl;

		auto key = cv::waitKey(1);
		if (key == 'q') { break; }
		DWORD loopTime = timeGetTime() - beginningOfLoop;
		if (loopTime < loopPeriod) { Sleep(loopPeriod - loopTime); }
	}

	*/
	std::cout << "Closing..." << std::endl;
	odcs.Close();
	threadProjection.join();
	return 0;
}