#ifndef _KinectApp_H_
#define _KinectApp_H_

#include <iostream>
#include <Kinect.h>
#include <atlbase.h>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

class KinectApp
{
public:
	//fields to get and store color image
	std::vector<BYTE> colorBuffer;

	//fields to get and store depth image
	std::vector<UINT16> depthBuffer;
	UINT16 depthMaxReliableDistance;
	UINT16 depthMinReliableDistance;

	//fields to store ir data
	std::vector<UINT16> infraredBuffer;

	//fields to store body index
	std::vector<BYTE> bodyIndexBuffer;

private:

	CComPtr<IKinectSensor> kinect;

	CComPtr<ICoordinateMapper> coordinateMapper = nullptr;

	CComPtr<IColorFrameReader> colorFrameReader;

	CComPtr<IDepthFrameReader> depthFrameReader;

	CComPtr<IInfraredFrameReader> infraredFrameReader;

	CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;

	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;

	int depthWidth;
	int depthHeight;

	int infraredWidth;
	int infraredHeight;
	int bodyIndexWidth;
	int bodyIndexHeight;
	cv::Scalar colors[6];

public:
	
	void initialize();

	HRESULT getColorBuffer();
	
	int getColorWidth();

	int getColorHeight();

	HRESULT getDepthBuffer();

	int getDepthWidth();

	int getDepthHeight();

	HRESULT getInfraredBuffer();

	int getInfraredWidth();

	int getInfraredHeight();

	HRESULT getBodyIndexBuffer();

	UINT16 getDepthAtColorPixel(int x, int y);

	CameraSpacePoint getPositionAtColorPixel(int x, int y);

	CameraSpacePoint getPositionAtDepthPixel(int x, int y);

	bool isReliableDepth(int depth);

	bool isReliablePosition(CameraSpacePoint pos);

	bool isInsideColorView(int x, int y);

	bool isInsideDepthView(int x, int y);
};

#endif