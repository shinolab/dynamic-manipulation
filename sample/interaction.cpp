#include "odcs.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{
	std::ofstream ofs("log.csv");
	//ofs << "t, x, y, z, isTracked" << std::endl;
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 200, 1500))));
	Eigen::Affine3f affineKinect2Global = odcs.ods.getAffineKinect2Global();
	std::cout << affineKinect2Global * Eigen::Vector3f(0, 0, 1000) << std::endl;
	//odcs.StartControl(objPtrs);
	
	//interaction sequence
	KinectApp app; 
	app.initialize();

	//initialize view


	bool isGrabbed = false;
	DWORD timeGrabbed;
	const float distThreshold = 200;
	const int timeThreshold = 3000;
	while (1)
	{
		app.getDepthBuffer();
		odcs.ods.DeterminePositionByDepthWithROI(objPtrs[0]);
		cv::Mat grayView(app.getDepthHeight(), app.getDepthWidth(), CV_8UC1);
		cv::Mat depthImage(app.getDepthHeight(), app.getDepthWidth(), CV_16UC1, &app.depthBuffer[0]);
		depthImage.convertTo(grayView, CV_8UC1, 255.0 / (float)app.depthMaxReliableDistance, 0);
		cv::Mat view(app.getDepthHeight(), app.getDepthWidth(), CV_8UC3);
		cv::cvtColor(grayView, view, CV_GRAY2BGR);

		HRESULT hr = app.getBodies();
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
					cv::circle(view, cv::Point(dspHandRight.X, dspHandRight.Y), 10, cv::Scalar(0, 0, 255));
					cv::circle(view, cv::Point(dspHandLeft.X, dspHandLeft.Y), 10, cv::Scalar(0, 255, 0));
					Eigen::Vector3f positionHandRight =
						affineKinect2Global * Eigen::Vector3f(1000 * cspHandRight.X, 1000 * cspHandRight.Y, 1000 * cspHandRight.Z);
					Eigen::Vector3f positionHandLeft =
						affineKinect2Global * Eigen::Vector3f(1000 * cspHandLeft.X, 1000 * cspHandLeft.Y, 1000 * cspHandLeft.Z);
					float distR = (objPtrs[0]->getPosition() - positionHandRight).norm();
					float distL = (objPtrs[0]->getPosition() - positionHandLeft).norm();

					cv::putText(view, std::to_string(distR), cv::Point(0, 50), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255));
					cv::putText(view, std::to_string(distL), cv::Point(0, 100), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255));

					if (distR < distThreshold || distL < distThreshold)
					{
						if (isGrabbed)
						{
							if ((timeGetTime() - timeGrabbed) > timeThreshold)
							{
								if (distR < distThreshold)
								{
									objPtrs[0]->updateStatesTarget(positionHandRight, Eigen::Vector3f(0, 0, 0));
									cv::putText(view,
										"Following (R)",
										cv::Point(dspHandRight.X, dspHandRight.Y),
										CV_FONT_HERSHEY_COMPLEX,
										0.6,
										cv::Scalar(0, 0, 255)
									);
								}
								else
								{
									objPtrs[0]->updateStatesTarget(positionHandLeft, Eigen::Vector3f(0, 0, 0));
									cv::putText(view,
										"Following(L)",
										cv::Point(dspHandRight.X, dspHandRight.Y),
										CV_FONT_HERSHEY_COMPLEX,
										10,
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
				}
			}
		}
		cv::imshow("VIEW", view);
		auto key = cv::waitKey(1);
		if(key == 'q')
		{
			break;
		}
	}

	std::cout << "Press any key to close." << std::endl;
	getchar();
	//odcs.Close();
	return 0;
}
