#define SUBDISPLAY_WIDTH 1024
#define SUBDISPLAY_HEIGHT 768
#define MAINDISPLAY_WIDTH 1366
#define MAINDISPLAY_HEIGHT 768

#include <thread>
#include "odcs.hpp"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

void imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY)
{
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);

	HWND windowHandle = ::FindWindowA(NULL, windowName);

	if (windowHandle != NULL)
	{
		SetWindowLongPtrA(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtrA(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		ShowWindow(windowHandle, SW_MAXIMIZE);
		cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		SetWindowPos(windowHandle, NULL,
			posX, posY, image.cols, image.rows, SWP_FRAMECHANGED | SWP_NOZORDER);
	}
	cv::imshow(windowName, image);
}

void projectImageOnObject(const char* windowName, Eigen::Vector3f posKinect, cv::Mat image)
{
	cv::Point3f objectPosition(posKinect.x(), posKinect.y(), posKinect.z() - 75);
	std::vector<cv::Point3f> imagePoints3d = { objectPosition };
	//here comes conversion from object position to object points
	std::vector<cv::Point2f> imagePoints2d;
	cv::Mat internalParam = (cv::Mat_<float>(3, 3) <<
		770, 0, SUBDISPLAY_WIDTH / 2,
		0, 760, SUBDISPLAY_HEIGHT + 105,
		0, 0, 1);
	cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.02374, -0.002134, -3.1495);
	cv::Mat tvec = (cv::Mat_<float>(3, 1) << 57.4, -617.5, 189.95);

	cv::Point2f centerImage;
	cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, cv::Mat(), imagePoints2d);
	
	cv::Mat dst(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
	float scale = 0.5 * 1000/objectPosition.z;
	cv::Mat affine = (cv::Mat_<float>(2, 3) <<
		scale, 0, (imagePoints2d[0].x - 0.5 * scale * image.cols),
		0, scale, (imagePoints2d[0].y - 0.5 * scale * image.rows) + 10);
	cv::warpAffine(image, dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	/*
	std::vector<cv::Point> originalImageCorners = {cv::Point(0,0), cv::Point(img0.cols, 0), cv::Point(0, img0.rows), cv::Point(img0.cols, img0.rows)};
	cv::Mat perspectiveMat = cv::getPerspectiveTransform(originalImageCorners, imagePoints2d);
	cv::Mat img1;
	cv::perspectiveTransform(img0, img1, perspectiveMat);
	*/
	imshowPopUp("FULL2", dst, MAINDISPLAY_WIDTH, 0);
}

int main()
{
	//initialization
	odcs odcs;
	odcs.Initialize();
	Eigen::Affine3f affineKinect2Global = odcs.ods.getAffineKinect2Global();
	Eigen::Affine3f affineGlobal2Kinect = affineKinect2Global.inverse();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	cv::Mat image = cv::imread("img/letters.jpg");
	//start control thread.
	odcs.StartControl(objPtrs);

	//start projection thread.
	std::thread threadProjection([&image, &objPtrs, &affineGlobal2Kinect]{
		while (1)
		{
			Eigen::Vector3f objPosKinect = affineGlobal2Kinect * objPtrs[0]->getPositionTarget();
			projectImageOnObject("FULL", objPosKinect, image);
			auto key = cv::waitKey(1);
			if (key == 'q')
			{
				break;
			}
		}
	});

	//start interaction thread.
	//interaction sequence
	KinectApp app;
	app.initialize();

	//initialize view
	bool isGrabbed = false;
	DWORD timeGrabbed;
	const float distThreshold = 500;
	const int timeThreshold = 3000;
	std::cout << "initialization completed." << std::endl;
	while (1)
	{
		HRESULT hr = app.getBodies();
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
						float distR = (objPtrs[0]->getPosition() - positionHandRight).norm();
						float distL = (objPtrs[0]->getPosition() - positionHandLeft).norm();
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
										objPtrs[0]->updateStatesTarget(objPtrs[0]->getPosition(), Eigen::Vector3f(0, 0, 0));
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
										objPtrs[0]->updateStatesTarget(objPtrs[0]->getPosition(), Eigen::Vector3f(0, 0, 0));
										cv::putText(view,
											"Following(L)",
											cv::Point(dspHandRight.X, dspHandRight.Y),
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
		auto key = cv::waitKey(1);
		if (key == 'q')
		{
			break;
		}
	}
	odcs.Close();
	threadProjection.join();
	return 0;
}