#include <KinectApp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

int main()
{
	KinectApp app;
	app.initialize();
	//Step 1 : get depth & color data
	while (1)
	{
		HRESULT hrDepth = app.getDepthBuffer(); Sleep(30);
		HRESULT hrColor = app.getColorBuffer();
		if (SUCCEEDED(hrDepth) && SUCCEEDED(hrColor))
		{
			std::chrono::system_clock::time_point start, end;
			start = std::chrono::system_clock::now();
			//Step 2: convert bgra data to hsv data
			cv::Mat imgBgra(app.getColorHeight(), app.getColorWidth(), CV_8UC4, &app.colorBuffer[0]);
			cv::Mat imgHsv, imgBgr;
			cv::cvtColor(imgBgra, imgBgr, cv::COLOR_BGRA2BGR);
			cv::cvtColor(imgBgr, imgHsv, cv::COLOR_BGR2HSV);
			//convert hsv image to depth resolusion
			std::vector<ColorSpacePoint> colorSpacePoints(app.getDepthHeight() * app.getDepthWidth());
			app.coordinateMapper->MapDepthFrameToColorSpace(app.depthBuffer.size()
				, &app.depthBuffer[0]
				, colorSpacePoints.size()
				, &colorSpacePoints[0]
			);

			cv::Mat imgHsvDepthRes(app.getDepthHeight(), app.getDepthWidth(), CV_8UC3, cv::Scalar::all(0));
			cv::MatIterator_<cv::Vec3b> itrH = imgHsvDepthRes.begin<cv::Vec3b>();
			int count = 0;
			for (auto itr = colorSpacePoints.begin(); itr != colorSpacePoints.end(); itr++, itrH++)
			{
				//std::cout << "x: " << (*itr).X << "y: " << (*itr).Y << std::endl;
				if (((*itr).X < app.getColorWidth()) && ((*itr).Y < app.getColorHeight()) && ((*itr).X > 0) && ((*itr).Y > 0))
				{
					*itrH = imgHsv.at<cv::Vec3b>((*itr).Y, (*itr).X);
				}
			}
			cv::Mat imgDepth(app.getDepthHeight(), app.getDepthWidth(), CV_16UC1, &app.depthBuffer[0]);
			//Create HSV mask
			cv::Mat maskHand; cv::inRange(imgHsvDepthRes, cv::Scalar(0, 30, 70), cv::Scalar(50, 100, 255), maskHand);
			cv::Mat maskBalloon; cv::inRange(imgHsvDepthRes, cv::Scalar(100, 150, 100), cv::Scalar(140, 255, 255), maskBalloon);
			cv::Mat maskDepth; cv::inRange(imgDepth, cv::Scalar(500), cv::Scalar(1800), maskDepth);
			cv::bitwise_and(maskHand, maskDepth, maskHand);
			cv::bitwise_and(maskBalloon, maskDepth, maskBalloon);
			//cv::erode(maskHand, maskHand, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
			//cv::erode(maskBalloon, maskBalloon, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

			//Create XYZ image
			std::vector<CameraSpacePoint> cameraPoints(app.depthBuffer.size());
			app.coordinateMapper->MapDepthFrameToCameraSpace(app.depthBuffer.size(), &app.depthBuffer[0], cameraPoints.size(), &cameraPoints[0]);
			cv::Mat r(app.getDepthHeight(), app.getDepthWidth(), CV_32FC3);
			cv::MatIterator_<cv::Vec3f> itrR = r.begin<cv::Vec3f>() + 51200;
			for (auto itr = cameraPoints.begin() + 51200; itr != cameraPoints.end() - 51200; itr++, itrR++)
			{
				itrR[0] = 1000 * (*itr).X;
				itrR[1] = 1000 * (*itr).Y;
				itrR[2] = 1000 * (*itr).Z;
			}

			cv::Scalar rMean = cv::mean(r, maskBalloon);
			std::cout << "rMean:\n" << rMean << std::endl;
			rMean = cv::Scalar::all(0);
			cv::Mat drb = r - rMean;
			std::vector<cv::Mat> dr(3);
			cv::split(drb, dr);
			cv::Mat db = dr[0].mul(dr[0]) + dr[1].mul(dr[1]) + dr[2].mul(dr[2]);
			cv::Point locQueryPoint;

			cv::minMaxLoc(db, NULL, NULL, &locQueryPoint, (cv::Point *)0, maskHand);

			cv::Vec3f queryPoint = r.at<cv::Vec3f>(locQueryPoint);
			cv::Mat distHand = r - queryPoint;
			std::vector<cv::Mat> drh(3);
			cv::split(distHand, drh);
			cv::Mat dh = drh[0].mul(drh[0]) + drh[1].mul(drh[1]) + drh[2].mul(drh[2]);

			double distBalloonHand;
			cv::Point locMinDistPoint;
			cv::minMaxLoc(dh, &distBalloonHand, NULL, &locMinDistPoint, NULL, maskBalloon);
			cv::Vec3f ClosestPointBalloon = r.at<cv::Vec3f>(locMinDistPoint);
			auto posRel = queryPoint - ClosestPointBalloon; //balloon-to-hand
			end = std::chrono::system_clock::now();
			std::cout << "duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
			std::cout << posRel;
			std::cout << "distance: " << sqrt(distBalloonHand) << std::endl;
			cv::Mat disp; cv::cvtColor(imgHsvDepthRes, disp, cv::COLOR_HSV2BGR);
			cv::circle(disp, locQueryPoint, 5, cv::Scalar(255, 0, 0), 2);
			cv::circle(disp, locMinDistPoint, 5, cv::Scalar(0, 0, 255), 2);
			std::cout << "querypoint:\n" << locQueryPoint << std::endl << "closestpoint:\n" << locMinDistPoint << std::endl;
			cv::imshow("handmask", maskHand);
			cv::imshow("balloonmask", maskBalloon);
			cv::imshow("hsvDepthRes", imgHsvDepthRes);
			cv::imshow("result", disp);

			auto key = cv::waitKey(30);
			if (key == 'q')
			{
				break;
			}

		}
	}

	return 0;
}