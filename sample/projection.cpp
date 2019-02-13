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
	std::string logname = "20190212_profiles_p2";
	//initialization
	odcs odcs;
	odcs.Initialize();
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-4.0f), Eigen::Vector3f::Constant(-0.05f));

	Eigen::Affine3f affineKinect2Global = odcs.odsPtr->getAffineKinect2Global();
	Eigen::Affine3f affineGlobal2Kinect = affineKinect2Global.inverse();
	Eigen::Matrix3f dcmGlobal2Kinect = odcs.odsPtr->getDcmGlobal2Kinect();
	odcs.AddObject(Eigen::Vector3f(0, 0, 1200));
	//start control thread.
	odcs.StartControl();
	std::string projectorName = "projector1";
	projector proj(projectorName);
	cv::Mat phone0 = cv::imread("img/phone0.jpg");
	cv::Mat phone1 = cv::imread("img/phone1.jpg");
	cv::Mat phone2 = cv::imread("img/phone2.jpg");
	cv::Mat phone3 = cv::imread("img/phone3.jpg");
	cv::Mat smile = cv::imread("img/smile_gray.png");
	cv::Mat face = cv::imread("img/normal_gray.png");
	cv::Mat angry = cv::imread("img/angry_gray.png");
	cv::Mat persevering = cv::imread("img/persevering.png");
	std::cout << "images loaded." << std::endl;
	proj.setImage(cv::Mat::zeros(100, 100, CV_8UC1));

	//log thread
	do {
	}while (odcs.GetFloatingObject(0)->isTracked);
	std::thread threadLog([&odcs, &logname]() {
		std::ofstream ofs(logname + "_pos.csv");
		ofs << "t, x, y, z, vx,vy, vz, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt" << std::endl;
		while (1)
		{
			Eigen::Vector3f pos = odcs.GetFloatingObject(0)->getPosition();
			Eigen::Vector3f vel = odcs.GetFloatingObject(0)->getVelocity();
			Eigen::Vector3f posTgt = odcs.GetFloatingObject(0)->getPositionTarget();
			Eigen::Vector3f velTgt = odcs.GetFloatingObject(0)->getVelocityTarget();
			ofs << timeGetTime()/1000.f << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " 
				<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
				<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << std::endl;
			Sleep(30);
		}
	});

	//start projection thread.
	std::thread threadProjection([&](){
		int num_average = 1;
		Eigen::MatrixXf posBuffer(3, num_average);
		proj.CreateScreen();
		unsigned int colcount = 0;
		while (1)
		{
			DWORD initTimeLoop = timeGetTime();
			if (odcs.GetFloatingObject(0)->isTracked)
			{
				Eigen::Vector3f pos = affineGlobal2Kinect * (odcs.GetFloatingObject(0)->getPosition()+Eigen::Vector3f(-5, 0, 10));
				posBuffer.col(colcount%num_average) << pos;
				//Eigen::Vector3f vel = dcmGlobal2Kinect * odcs.GetFloatingObject(0)->getVelocity();
				colcount++;
			
				//projectImageOnObject("FULL", pos + vel * (timeGetTime() - odcs.GetFloatingObject(0)->lastDeterminationTime)/1000, image);
				proj.projectImageOnObject(posBuffer.rowwise().sum() / num_average, cv::Size(180,180), cv::Scalar::all(255)); // for VR LOGO

				
				auto key = cv::waitKey(1);
				if (key == 'q') { break; }
				int loopTime = (timeGetTime() - initTimeLoop);
				if (loopTime < 10) { Sleep(10 - loopTime); }
			}
		}
	});
	Sleep(10000);
	std::cout << "CALLING..." << std::endl;
	int phase = 0;
	DWORD switchedTime = timeGetTime();
	do{
		int period = 500;//ms
		if (timeGetTime() - switchedTime > period)
		{
			switch (phase)
			{
			case 0:
				proj.setImage(phone1); phase++;
				break;
			case 1:
				proj.setImage(phone2); phase++;
				break;
			case 2:
				proj.setImage(phone3); phase++;
				break;
			case 3:
				proj.setImage(phone0); phase = 0;
				std::cout << "Loop" << std::endl;
				break;
			default:
				break;
			}
			switchedTime = timeGetTime();
			Sleep(100);
			std::cout << "phase: " << phase << std::endl;
		}
	} while (odcs.GetFloatingObject(0)->getPosition().x() > -100);
	

	proj.setImage(face);

	Sleep(3000); // wait for hand to come in
	KinectApp app;
	app.initialize();
	bool tracking_succeeded = false;
	Eigen::Vector3f posHand;
	do
	{
		HRESULT hr = app.getBodies();
		if (SUCCEEDED(hr)) {
			for (auto itr = app.pBodies.begin(); itr != app.pBodies.end(); itr++)
			{
				BOOLEAN isTracked = false;
				HRESULT hrTrack = (*itr)->get_IsTracked(&isTracked);
				if (isTracked && SUCCEEDED(hrTrack))
				{
					std::vector<Joint> joints(JointType::JointType_Count);
					HRESULT hrJoints = (*itr)->GetJoints(JointType::JointType_Count, &joints[0]);
					if (SUCCEEDED(hrJoints)) {
						CameraSpacePoint cspHandLeft = joints[JointType::JointType_HandLeft].Position;
						posHand = affineKinect2Global* Eigen::Vector3f(1000 * cspHandLeft.X, 1000 * cspHandLeft.Y, 1000 * cspHandLeft.Z);
						if (odcs.odsPtr->isInsideWorkSpace(posHand)){
							tracking_succeeded = true;
						}
					}
				}
			}
		}
		Sleep(10);
	} while (!tracking_succeeded);
	std::cout << "User Found" << std::endl;

	std::thread bodyTrackingThread([&app, &logname, &affineKinect2Global]() {
		std::ofstream ofsBody(logname + "_body.csv");
		ofsBody << "t, x_hand, y_hand, z_hand" << std::endl;
		while (1) {
			HRESULT hr = app.getBodies();
			if (SUCCEEDED(hr)) {
				for (auto itr = app.pBodies.begin(); itr != app.pBodies.end(); itr++)
				{
					BOOLEAN isTracked = false;
					HRESULT hrTrack = (*itr)->get_IsTracked(&isTracked);
					if (isTracked && SUCCEEDED(hrTrack))
					{
						std::vector<Joint> joints(JointType::JointType_Count);
						HRESULT hrJoints = (*itr)->GetJoints(JointType::JointType_Count, &joints[0]);
						if (SUCCEEDED(hrJoints)) {
							CameraSpacePoint cspHandLeft = joints[JointType::JointType_HandLeft].Position;
							Eigen::Vector3f posHandvec = affineKinect2Global * Eigen::Vector3f(1000 * cspHandLeft.X, 1000 * cspHandLeft.Y, 1000 * cspHandLeft.Z);
							ofsBody << timeGetTime() / 1000.f << ", " << posHandvec.x() << ", " << posHandvec.y() << ", " << posHandvec.z() << std::endl;
						}
					}
				}
			}
			Sleep(30);
		}		
	});
	proj.setImage(smile);
	profileBangBang profileGentleF(5.0f, timeGetTime() / 1000.f, odcs.GetFloatingObject(0)->getPositionTarget(), posHand);
	DWORD gentleInit = timeGetTime();
	do {
		odcs.GetFloatingObject(0)->updateStatesTarget(profileGentleF.posTgt(), profileGentleF.velTgt(), profileGentleF.accelTgt());
		Sleep(10);
	} while (timeGetTime() - gentleInit < 5000);
	proj.setImage(face);

	profileBangBang profileGentleB(5.0f, timeGetTime() / 1000.f, posHand, Eigen::Vector3f(0, 0, 1200));
	DWORD gentleInitB = timeGetTime();
	do {
		odcs.GetFloatingObject(0)->updateStatesTarget(profileGentleB.posTgt(), profileGentleB.velTgt(), profileGentleB.accelTgt());
		Sleep(10);
	} while (timeGetTime() - gentleInitB < 6000);
	Sleep(2000);

	tracking_succeeded = false;
	do
	{
		HRESULT hr = app.getBodies();
		if (SUCCEEDED(hr)) {
			for (auto itr = app.pBodies.begin(); itr != app.pBodies.end(); itr++)
			{
				BOOLEAN isTracked = false;
				HRESULT hrTrack = (*itr)->get_IsTracked(&isTracked);
				if (isTracked && SUCCEEDED(hrTrack))
				{
					std::vector<Joint> joints(JointType::JointType_Count);
					HRESULT hrJoints = (*itr)->GetJoints(JointType::JointType_Count, &joints[0]);
					if (SUCCEEDED(hrJoints)) {
						CameraSpacePoint cspHandLeft = joints[JointType::JointType_HandLeft].Position;
						posHand = affineKinect2Global * Eigen::Vector3f(1000 * cspHandLeft.X, 1000 * cspHandLeft.Y, 1000 * cspHandLeft.Z);
						if (odcs.odsPtr->isInsideWorkSpace(posHand)) {
							tracking_succeeded = true;
						}
					}
				}
			}
		}
		Sleep(10);
	} while (!tracking_succeeded);
	proj.setImage(angry);
	std::cout << "User Found" << std::endl;

	profileBang profileAngryF(1.0f, timeGetTime() / 1000.f, odcs.GetFloatingObject(0)->getPositionTarget(), posHand);
	DWORD angryInit = timeGetTime();
	do {
		odcs.GetFloatingObject(0)->updateStatesTarget(profileAngryF.posTgt(), profileAngryF.velTgt(), profileAngryF.accelTgt());
		Sleep(10);
	} while (timeGetTime() - angryInit < 2000);
	proj.setImage(face);


	/*
	std::ofstream ofs("20181129_xy1300retry2.csv");
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

	*/
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