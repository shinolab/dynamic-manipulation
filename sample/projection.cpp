#define SUBDISPLAY_WIDTH 3840
#define SUBDISPLAY_HEIGHT 2160
#define MAINDISPLAY_WIDTH 1920
#define MAINDISPLAY_HEIGHT 1080

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
#include <opencv2/core/eigen.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

class projector {
private:
	cv::Mat internalParam;
	cv::Mat distCoeffs;
	cv::Mat rvec;
	cv::Mat tvec;

public:
	projector();
	
	void imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY);

	void projectImageOnObject(const char* windowName, Eigen::Vector3f posKinect, cv::Mat image, cv::Size sizeReal);

	cv::Mat GenerateLandoltImage(float acuity, float distanceviewer, float distanceProj, int direction = 0);

	Eigen::Affine3f affineKinect2Projector();

	void projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints);
};

projector::projector()
{
	internalParam = (cv::Mat_<float>(3, 3) <<
		5933, 0, 1398,
		0, 5998, 2018,
		0, 0, 1);
	distCoeffs = (cv::Mat_<float>(1, 5) << -0.0355, 0.0376, -0.0167, 0.00095, 0);
	rvec = (cv::Mat_<float>(3, 1) << 0.825, 0.0783, 3.01);
	tvec = (cv::Mat_<float>(3, 1) << -395.46, -406.26, 1095.4);
}

void projector::imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY)
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

Eigen::Affine3f projector::affineKinect2Projector()
{
	Eigen::Vector3f rodrigues;
	Eigen::Vector3f translation;
	cv::cv2eigen(rvec, rodrigues);
	cv::cv2eigen(tvec, translation);
	return Eigen::Translation3f(translation)*Eigen::AngleAxisf(rodrigues.norm(), rodrigues.normalized());
}

//size should be specified in [mm]
void projector::projectImageOnObject(const char* windowName, Eigen::Vector3f posKinect, cv::Mat image, cv::Size sizeReal)
{
	float distance = (affineKinect2Projector()*posKinect).z();
	cv::Point3f objectPosition(posKinect.x(), posKinect.y(), posKinect.z());
	std::vector<cv::Point3f> imagePoints3d = { objectPosition };
	//here comes conversion from object position to object points
	std::vector<cv::Point2f> imagePoints2d;
	cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, distCoeffs, imagePoints2d);
	cv::Mat dst(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
	float fx = internalParam.at<float>(0, 0);
	float fy = internalParam.at<float>(1, 1);
	cv::Rect roi(cv::Point(0 * image.cols, 0), cv::Point(1.0 * image.cols, 1.0 * image.rows));
	cv::Mat affine = (cv::Mat_<float>(2, 3) <<
		fx * sizeReal.width / distance /image.cols, 0, ((int)(imagePoints2d[0].x - 0.5 * sizeReal.width * fx / distance)),
		0, fy * sizeReal.height / distance / image.rows, ((int)(imagePoints2d[0].y - 0.5 * sizeReal.height * fy / distance)));
	cv::warpAffine(image(roi), dst, affine, dst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	cv::imshow(windowName, dst);
}

void projector::projectPoints(const std::vector<cv::Point3f> &objectPoints, std::vector<cv::Point2f> &imagePoints)
{
	cv::projectPoints(objectPoints, rvec, tvec, internalParam, distCoeffs, imagePoints);
}

cv::Mat projector::GenerateLandoltImage(float acuity, float distanceViewer, float distanceProj, int direction)
{
	//cv::Mat imgLandolt(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC1, cv::Scalar(255));

	float slitWidthReal = distanceViewer * tanf(1 / acuity * M_PI / 180 / 60);
	
	float fx = internalParam.at<float>(0, 0);
	float fy = internalParam.at<float>(1, 1);
	
	cv::Mat imgLandolt(5 * slitWidthReal * fx / distanceProj, 5 * slitWidthReal / distanceProj * fy, CV_8UC1, cv::Scalar(255));
	cv::ellipse(imgLandolt, cv::RotatedRect(cv::Point(0, 0), cv::Point(imgLandolt.cols, 0), cv::Point(imgLandolt.cols, imgLandolt.rows)), cv::Scalar(0), -1);
	cv::ellipse(imgLandolt, cv::RotatedRect(cv::Point(imgLandolt.cols / 5, imgLandolt.rows / 5), cv::Point(4 * imgLandolt.cols / 5, imgLandolt.rows / 5), cv::Point(4 * imgLandolt.cols / 5, 4 * imgLandolt.rows / 5)), cv::Scalar(255), -1);

	cv::Rect slit;
	switch (direction) {
	case 0: //left
		slit = cv::Rect(0, 2 * imgLandolt.rows / 5, imgLandolt.cols / 2 + 1, imgLandolt.rows / 5);
		break;
	case 1: //right
		slit = cv::Rect(imgLandolt.cols / 2, 2 * imgLandolt.rows / 5, imgLandolt.cols / 2 + 1, imgLandolt.rows / 5);
		break;
	case 2: //up
		slit = cv::Rect(2 * imgLandolt.cols / 5, 0, imgLandolt.cols / 5, imgLandolt.rows / 2 + 1);
		break;
	default: //down
		slit = cv::Rect(2 * imgLandolt.cols / 5, imgLandolt.rows / 2, imgLandolt.cols / 5, imgLandolt.rows / 2 + 1);
		break;
	}
	cv::rectangle(imgLandolt, slit, cv::Scalar(255), -1);
	return imgLandolt;
}

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
		5933, 0, 1398,
		0, 5998, 2018,
		0, 0, 1);
	cv::Mat distCoeff = (cv::Mat_<float>(1, 5) << -0.0355, 0.0376, -0.0167, 0.00095, 0);
	cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.825, 0.0783, 3.01);
	cv::Mat tvec = (cv::Mat_<float>(3, 1) << -395.46, -406.26, 1095.4);

	cv::Point2f centerImage;
	cv::projectPoints(imagePoints3d, rvec, tvec, internalParam, distCoeff, imagePoints2d);
	
	cv::Mat dst(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
	float scale = 5.0 * 1000.0 / objectPosition.z;
	float zscale = 1000.0 / objectPosition.z;
	cv::Rect roi(cv::Point(0 * image.cols, 0),cv::Point(1.0 * image.cols, 1.0 * image.rows));
	cv::Mat affine = (cv::Mat_<float>(2, 3) <<
		scale, 0, ((int)(imagePoints2d[0].x - 0.5 * scale * image.cols + 1.0 * zscale)),
		0, scale, ((int)(imagePoints2d[0].y - 0.5 * scale * image.rows - 1.0 * zscale)));
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


int main()
{
	//for test
	projector proj;
	
	Eigen::Vector3f p1e = (proj.affineKinect2Projector()*Eigen::Vector3f(0, 0, 1000));
	Eigen::Vector3f p2e = (proj.affineKinect2Projector()*Eigen::Vector3f(200, 0, 1000));
	Eigen::Vector3f p3e = (proj.affineKinect2Projector()*Eigen::Vector3f(0, 200, 1000));
	Eigen::Vector3f p4e = (proj.affineKinect2Projector()*Eigen::Vector3f(0, 0, 1200));

	cv::Mat p1, p2, p3, p4;
	cv::eigen2cv(p1e, p1);
	cv::eigen2cv(p2e, p2);
	cv::eigen2cv(p3e, p3);
	cv::eigen2cv(p4e, p4);
	std::cout << p1 << std::endl;
	std::cout << p2 << std::endl;
	std::cout << p3 << std::endl;
	std::cout << p4 << std::endl;
	cv::Point3f p1p = p1;
	cv::Point3f p2p = p2; 
	cv::Point3f p3p = p3;
	cv::Point3f p4p = p4;

	std::vector<cv::Point3f> points3f = { p1p, p2p, p3p, p4p };
	std::vector<cv::Point2f> points2f;
	proj.projectPoints(points3f, points2f);
	
	for (auto itr = points2f.begin(); itr != points2f.end(); itr++)
	{
		std::cout << std::distance(points2f.begin(), itr) << ": " << (*itr).x << ", " << (*itr).y << std::endl;
	}

	cv::Mat imgL1 = proj.GenerateLandoltImage(0.1, 2000, 2000, 1);
	cv::imshow("Landolt C1", imgL1);
	cv::Mat imgL2 = proj.GenerateLandoltImage(0.1, 2000, 2000, 2);
	cv::imshow("Landolt C2", imgL2);
	cv::Mat imgL4 = proj.GenerateLandoltImage(0.1, 2000, 2000, 3);
	cv::imshow("Landolt C4", imgL4);

	cv::Mat fontTest(500, 500, CV_8UC3, cv::Scalar::all(255));
	cv::addText(fontTest, "testTEST", cv::Point(250, 250), cv::fontQt("Arial", 20));
	cv::addText(fontTest, "testTEST", cv::Point(0, 250), cv::fontQt("Times New Roman", 20));
	cv::rectangle(fontTest, cv::Point(250, 250 - 30), cv::Point(410, 250), cv::Scalar::all(0));
	cv::addText(fontTest, "‚Ä‚·‚ÆƒeƒXƒg", cv::Point(0, 300), cv::fontQt("MS PMincho", 20));
	cv::addText(fontTest, "testTest", cv::Point(250, 300), cv::fontQt("tekito", 20));

	cv::imshow("cvQtTest", fontTest);

	cv::waitKey(0);
	return 0;
	//initialization
	odcs odcs;
	odcs.Initialize();
	Eigen::Affine3f affineKinect2Global = odcs.ods.getAffineKinect2Global();
	Eigen::Affine3f affineGlobal2Kinect = affineKinect2Global.inverse();
	Eigen::Matrix3f dcmGlobal2Kinect = odcs.ods.getDcmGlobal2Kinect();
	odcs.AddObject(Eigen::Vector3f(0, 0, 1300));
	cv::Mat image = cv::imread("img/shinolab3.jpg");
	cv::VideoCapture cap("video/bird2.mp4");
	//start control thread.
	odcs.StartControl();

	//start projection thread.
	std::thread threadProjection([&image, &cap, &odcs, &affineGlobal2Kinect, &dcmGlobal2Kinect](){
		cv::Mat blank(SUBDISPLAY_HEIGHT, SUBDISPLAY_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
		imshowPopUp("FULL", blank, MAINDISPLAY_WIDTH, 0);
		const int num_frame = cap.get(cv::CAP_PROP_FRAME_COUNT);
		int count_frame = 0;
		int num_average = 3;
		Eigen::MatrixXf posBuffer(3, num_average);
		unsigned int colcount = 0;
		while (1)
		{
			DWORD initTimeLoop = timeGetTime();
			if (odcs.GetFloatingObject(0)->isTracked)
			{
				Eigen::Vector3f pos = affineGlobal2Kinect * odcs.GetFloatingObject(0)->getPosition();
				posBuffer.col(colcount%num_average) << pos;
				//Eigen::Vector3f vel = dcmGlobal2Kinect * odcs.GetFloatingObject(0)->getVelocity();
				colcount++;
				//cap >> image;
				//projectImageOnObject("FULL", pos + vel * (timeGetTime() - odcs.GetFloatingObject(0)->lastDeterminationTime)/1000, image);
				projectImageOnObject("FULL", posBuffer.rowwise().sum() / num_average, image);
				//std::cout << posBuffer.rowwise().sum().transpose() << std::endl; ;
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

	std::ofstream ofs("20180918_projection_log2.csv");
	float lengthX = 300;
	float offsetX = 0;
	float offsetY = 0;
	float offsetZ = 1300;
	int period = 6000;
	Sleep(20000);
	ofs << "time, x, y, z, v_x, v_y, v_z, I_x, I_y, I_z, x_tgt, y_tgt, z_tgt, v_x, v_y, v_z, u0, u1, u2, u3, u4" << std::endl;
	DWORD initialTime = timeGetTime();
	DWORD loopPeriod = 30;
	while ((timeGetTime()-initialTime) < 4 * period)
	{
		DWORD beginningOfLoop = timeGetTime();
		float phase = 2 * M_PI * ((timeGetTime() - initialTime) % period) / (float)period;
		DWORD t = beginningOfLoop - initialTime;
		float x, y, z, vx, vy, vz;
		switch(t / period)
		{
		case 0:
			vx = 0; vy = 0; vz = 0;
			x = lengthX;  y = lengthX ; z = offsetZ;
			break;
		case 1:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = 0; z = offsetZ;
			break;
		case 2:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = 0; z = offsetZ-300;
			break;
		case 3:
			vx = 0; vy = 0; vz = 0;
			x = 0;  y = 0; z = offsetZ+200;
			break;
		default:
			vx = 0; vy = 0; vz = 0;
			x = 0; y = 0; z = offsetZ;
			break;
		}
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
	odcs.GetFloatingObject(0)->updateStatesTarget(Eigen::Vector3f(offsetX, offsetY, offsetZ), Eigen::Vector3f(0, 0, 0));
	std::cout << "transition to interaction mode." << std::endl;
	
	//start interaction thread.
	//interaction sequence
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
	std::cout << "Closing..." << std::endl;
	odcs.Close();
	threadProjection.join();
	return 0;
}