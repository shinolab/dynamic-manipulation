#define SUBDISPLAY_WIDTH 1024
#define SUBDISPLAY_HEIGHT 768
#define MAINDISPLAY_WIDTH 1366
#define MAINDISPLAY_HEIGHT 768

#include <thread>
#include <fstream>
#include "odcs.hpp"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

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
	cv::Point3f objectPosition(posKinect.x(), posKinect.y(), posKinect.z());
	std::vector<cv::Point3f> imagePoints3d = { objectPosition };
	//here comes conversion from object position to object points
	std::vector<cv::Point2f> imagePoints2d;
	cv::Mat internalParam = (cv::Mat_<float>(3, 3) <<
		734, 0, 510,
		0, 741, 900,
		0, 0, 1);
	cv::Mat distCoeff = (cv::Mat_<float>(1, 5) << (int)0.03965, (int)-0.0151, (int)0.01114, (int)-0.0003963, 0);
	cv::Mat rvec = (cv::Mat_<float>(3, 1) << (int)-0.0005831, (int)-0.04041, (int)3.1242);
	cv::Mat tvec = (cv::Mat_<float>(3, 1) << (int)25.1, (int)-764.8, (int)154.7);

	cv::Point2f centerImage;
	cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, distCoeff, imagePoints2d);
	
	cv::Mat dst(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
	float scale = 1.0;  1.0 * 1000.0 / objectPosition.z;
	float zscale = 1000.0 / objectPosition.z;
	cv::Rect roi(cv::Point(0 * image.cols, 0),cv::Point(1.0 * image.cols, 1.0 * image.rows));
	cv::Mat affine = (cv::Mat_<float>(2, 3) <<
		(int)scale, 0, ((int)(imagePoints2d[0].x - 0.5 * scale * image.cols + 0 * zscale)),
		0, (int)scale, ((int)(imagePoints2d[0].y - 0.5 * scale * image.rows - 0 * zscale)));
	cv::warpAffine(image(roi), dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	/*
	std::vector<cv::Point> originalImageCorners = {cv::Point(0,0), cv::Point(img0.cols, 0), cv::Point(0, img0.rows), cv::Point(img0.cols, img0.rows)};
	cv::Mat perspectiveMat = cv::getPerspectiveTransform(originalImageCorners, imagePoints2d);
	cv::Mat img1;
	cv::perspectiveTransform(img0, img1, perspectiveMat);
	*/
	//imshowPopUp(windowName, dst, MAINDISPLAY_WIDTH, 0);
	cv::imshow(windowName, dst);
}

void runInteractionLoop(Eigen::Affine3f affineKinect2Global, std::vector<FloatingObjectPtr> objPtrs)
{
	KinectApp app;
	app.initialize();
	bool isGrabbed = false;
	DWORD timeGrabbed;
	const float distThreshold = 500;
	const int timeThreshold = 3000;
	std::cout << "initialization completed." << std::endl;
	while (1)
	{
		Sleep(30);
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
						cv::waitKey(1);
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
}

int main()
{
	std::ofstream ofs_basic("20180806_averagingTime7_log.csv");
	ofs_basic << "frame#, x, y, z" << std::endl;
	//initialization
	odcs odcs;
	odcs.Initialize();
	Eigen::Affine3f affineKinect2Global = odcs.ods.getAffineKinect2Global();
	Eigen::Affine3f affineGlobal2Kinect = affineKinect2Global.inverse();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1600))));
	cv::Mat image = cv::imread("img/shinolab2.jpg");
	cv::VideoCapture cap("video/bird2.mp4");
	//start control thread.
	odcs.StartControl(objPtrs);

	//start projection thread.
	std::thread threadProjection([&image, &cap, &objPtrs, &affineGlobal2Kinect, &ofs_basic]{
		cv::Mat blank(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
		imshowPopUp("FULL", blank, MAINDISPLAY_WIDTH, 0);
		const int num_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
		int count_frame = 0;
		while (1)
		{
			int num_average = 7
;
			Eigen::Vector3f objPosKinect(0, 0, 0);
			for (int iAve = 0; iAve < num_average; iAve++)
			{
				objPosKinect += affineGlobal2Kinect * objPtrs[0]->getPosition();
				cv::waitKey(25);
			}
			objPosKinect /= num_average;
			ofs_basic << count_frame << "," << objPosKinect.x() << ", " << objPosKinect.y() << ", " << objPosKinect.z() << std::endl;
			//cap >> image;
			projectImageOnObject("FULL", objPosKinect, image);
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
			if (key == 'q')
			{
				break;
			}
		}
	});
	getchar();
	float lengthX = 400;
	float offsetX = 0;
	float offsetY = 0;
	float offsetZ = 1600;
	int period = 10000;
	Sleep(40000);
	std::ofstream ofs("20180723_projection_log_step5.csv");
	ofs << "time, x, y, z, v_x, v_y, v_z, I_x, I_y, I_z, x_tgt, y_tgt, z_tgt, v_x, v_y, v_z, u0, u1, u2, u3, u4" << std::endl;
	DWORD initialTime = timeGetTime();
	DWORD loopPeriod = 30;
	while ((timeGetTime()-initialTime) < 2 * period)
	{
		DWORD beginningOfLoop = timeGetTime();
		//float phase = 2 * M_PI * ((timeGetTime() - initialTime) % period) / (float)period;
		DWORD t = beginningOfLoop - initialTime;
		float x, y, z, vx, vy, vz;
		switch(t / period)
		{
		case 0:
			vx = 0; vy = 0; vz = 0;
			x = lengthX;  y = 0; z = offsetZ; 
			break;
		case 1:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = lengthX; z = offsetZ;
			break;
		case 2:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = 0; z = offsetZ + lengthX;
			break;
		case 3:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = 0; z = offsetZ;
			break;
		default:
			vx = 0; vy = 0; vz = 0;
			x = offsetX; y = offsetY; z = offsetZ;
		}
		objPtrs[0]->updateStatesTarget(Eigen::Vector3f(x, y, z), Eigen::Vector3f(vx, vy, vz));
		Eigen::Vector3f currentPosition = objPtrs[0]->getPosition();
		ofs << timeGetTime() - initialTime << ", " << currentPosition.x() << ", " << currentPosition.y() << ", " << currentPosition.z() << ", "
			<< x << ", " << y << "," << z << ", " << vx << ", " << vy << ", " << vz << std::endl;
		DWORD loopTime = timeGetTime() - beginningOfLoop;
		if (loopTime < loopPeriod)
		{
			Sleep(loopPeriod - loopTime);
		}
	}
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(offsetX, offsetY, offsetZ), Eigen::Vector3f(0, 0, 0));
	std::cout << "transition to interaction mode." << std::endl;
	//start interaction thread.
	//interaction sequence
	KinectApp app;
	app.initialize();
	bool isGrabbed = false;
	DWORD timeGrabbed;
	const float distThreshold = 500;
	const int timeThreshold = 1500;
	std::cout << "initialization completed." << std::endl;
	while (timeGetTime() - initialTime < 90000)
	{
		DWORD beginningOfLoop = timeGetTime();
		HRESULT hr = app.getBodies();
		Eigen::Vector3f objectPosition = objPtrs[0]->getPosition();
		Eigen::Vector3f targetPosition = objPtrs[0]->getPositionTarget();

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
		ofs << timeGetTime() - initialTime
			<< ", " << objectPosition.x() << ", " << objectPosition.y() << ", " << objectPosition.z()
			<< ", " << targetPosition.x() << ", " << targetPosition.y() << ", " << targetPosition.z() << std::endl;

		auto key = cv::waitKey(1);
		if (key == 'q')
		{
			break;
		}
		DWORD loopTime = timeGetTime() - beginningOfLoop;
		if (loopTime < loopPeriod)
		{
			Sleep(loopPeriod - loopTime);
		}
	}
	std::cout << "Closing..." << std::endl;
	odcs.Close();
	threadProjection.join();
	return 0;
}