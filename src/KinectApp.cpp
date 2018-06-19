#ifndef kinectApp_
#define kinectApp_
#include <atlbase.h>
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>

#include "KinectApp.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
			    }

void KinectApp::initialize()
{
	// デフォルトのKinectを取得する
	ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
	
	// Kinectを開く
	ERROR_CHECK(kinect->Open());

	BOOLEAN isOpen = false;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen){
		throw std::runtime_error("Kinectが開けません");
	}

	//-----prepare color reader and storage-----
	CComPtr<IColorFrameSource> colorFrameSource;
	kinect->get_ColorFrameSource(&colorFrameSource);

	kinect->get_CoordinateMapper(&coordinateMapper);

	CComPtr<IFrameDescription> colorFrameDescription;
	colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription);
	colorFrameDescription->get_Height(&colorHeight);
	colorFrameDescription->get_Width(&colorWidth);
	colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);

	colorBuffer.resize(colorHeight * colorWidth * colorBytesPerPixel);

	colorFrameSource->OpenReader(&colorFrameReader);

	//-----prepare depth reader and storage -----
	CComPtr<IDepthFrameSource> depthFrameSource;
	kinect->get_DepthFrameSource(&depthFrameSource);

	CComPtr<IFrameDescription> depthFrameDescription;
	depthFrameSource->get_FrameDescription(&depthFrameDescription);
	depthFrameDescription->get_Height(&depthHeight);
	depthFrameDescription->get_Width(&depthWidth);
	depthFrameSource->get_DepthMaxReliableDistance(&depthMaxReliableDistance);
	depthFrameSource->get_DepthMinReliableDistance(&depthMinReliableDistance);

	depthFrameSource->OpenReader(&depthFrameReader);

	depthBuffer.resize(depthHeight * depthWidth);

	//-----prepare ir storage-----

	CComPtr<IInfraredFrameSource> infraredFrameSource;
	kinect->get_InfraredFrameSource(&infraredFrameSource);
	infraredFrameSource->OpenReader(&infraredFrameReader);

	CComPtr<IFrameDescription> infraredFrameDescription;
	infraredFrameSource->get_FrameDescription(&infraredFrameDescription);
	infraredFrameDescription->get_Height(&infraredHeight);
	infraredFrameDescription->get_Width(&infraredWidth);
	infraredBuffer.resize(infraredHeight * infraredWidth);

	//-----prepare body index -----

	CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
	kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource);
	bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader);

	CComPtr<IFrameDescription> bodyIndexFrameDescription;
	bodyIndexFrameSource->get_FrameDescription(&bodyIndexFrameDescription);
	bodyIndexFrameDescription->get_Height(&bodyIndexHeight);
	bodyIndexFrameDescription->get_Width(&bodyIndexWidth);

	bodyIndexBuffer.resize(bodyIndexHeight * bodyIndexWidth);

	//-----initialization for body info-----
	CComPtr<IBodyFrameSource> pBodyFrameSource;
	kinect->get_BodyFrameSource(&pBodyFrameSource);
	pBodyFrameSource->OpenReader(&bodyFrameReader);
	pBodies.resize(BODY_COUNT);
}

HRESULT KinectApp::getColorBuffer()
	{
		CComPtr<IColorFrame> colorFrame;
		HRESULT hResult = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (SUCCEEDED(hResult))
		{
			colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);
		}
		return hResult;
	}

int KinectApp::getColorHeight()
	{
		return colorHeight;
	}

int KinectApp::getColorWidth()
	{
		return colorWidth;
	}

HRESULT KinectApp::getDepthBuffer()
	{
		CComPtr<IDepthFrame> depthFrame;
		HRESULT hResult = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (SUCCEEDED(hResult))
		{
			depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
		}
		return hResult;
	}

int KinectApp::getDepthHeight()
{
	return depthHeight;
}

int KinectApp::getDepthWidth()
{
	return depthWidth;
}

HRESULT KinectApp::getInfraredBuffer()
	{
		CComPtr<IInfraredFrame> infraredFrame;
		HRESULT hResult = infraredFrameReader->AcquireLatestFrame(&infraredFrame);
		if (SUCCEEDED(hResult))
		{
			infraredFrame->CopyFrameDataToArray(infraredBuffer.size(), &infraredBuffer[0]);
		}
		return hResult;
	}

int KinectApp::getInfraredHeight()
{
	return infraredHeight;
}

int KinectApp::getInfraredWidth()
{
	return infraredWidth;
}

HRESULT KinectApp::getBodyIndexBuffer()
	{
		CComPtr<IBodyIndexFrame> bodyIndexFrame;
		HRESULT hr = bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame);
		if (FAILED(hr))
		{
			return hr;
		}
		bodyIndexFrame->CopyFrameDataToArray(bodyIndexBuffer.size(), &bodyIndexBuffer[0]);
		return hr;
	}

UINT16 KinectApp::getDepthAtColorPixel(int x, int y)
{
	std::vector<DepthSpacePoint> depthSpace(colorWidth * colorHeight);
	coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthSpace.size(), &depthSpace[0]);

	int index = y * colorWidth + x;
	int xD = (int)depthSpace[index].X;
	int yD = (int)depthSpace[index].Y;
	UINT16 depth = depthBuffer[yD * depthWidth + xD];
	return depth;
}

CameraSpacePoint KinectApp::getPositionAtColorPixel(int x, int y)
{
	std::vector<CameraSpacePoint> cameraSpace(colorWidth * colorHeight);
	coordinateMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], cameraSpace.size(), &cameraSpace[0]);
	int index = y * colorWidth + x;
	return cameraSpace[index];
}

CameraSpacePoint KinectApp::getPositionAtDepthPixel(int x, int y)
{
	DepthSpacePoint depthPoint; depthPoint.X = x; depthPoint.Y = y;
	int index = y * depthWidth + x;
	UINT16 depth = depthBuffer[index];
	CameraSpacePoint position;
	coordinateMapper->MapDepthPointToCameraSpace(depthPoint, depth, &position);
	return position;
}

DepthSpacePoint KinectApp::convertPositionToDepthPixel(CameraSpacePoint csp)
{
	DepthSpacePoint dsp;
	coordinateMapper->MapCameraPointToDepthSpace(csp, &dsp);
	return dsp;
}

bool KinectApp::isReliableDepth(int depth)
{
	return (depth < depthMaxReliableDistance) && (depth > depthMinReliableDistance);
}

bool KinectApp::isReliablePosition(CameraSpacePoint pos)
{
	return isReliableDepth(1000 * pos.Z);
}

bool KinectApp::isInsideColorView(int x, int y)
{
	return (0 < x) && (x < colorWidth) && (0 < y) && (y < colorHeight);
}

bool KinectApp::isInsideDepthView(int x, int y)
{
	return (0 < x) && (x < depthWidth) && (0 < y) && (y < depthHeight);
}

HRESULT KinectApp::getBodies()
{
	CComPtr<IBodyFrame> pBodyFrame = nullptr;
	HRESULT hr = bodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hr))
	{
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, &pBodies[0]);
	}
	return hr;
}

#endif