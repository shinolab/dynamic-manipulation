#include "KinectUtility.hpp"
#include "Kinect.h"
#include <sstream>
#include <iostream>

namespace KinectUtility {

#pragma region Depth Manager

	KinectDepthManager::KinectDepthManager(){
		CComPtr<IKinectSensor> pKinect;
		error_check(::GetDefaultKinectSensor(&pKinect));
		BOOLEAN isOpen;
		error_check(pKinect->get_IsOpen(&isOpen));
		if (!isOpen) {
			throw std::runtime_error("Kinect is not opened.");
		}
		
		CComPtr<IDepthFrameSource> pDepthFrameSource;
		pKinect->get_DepthFrameSource(&pDepthFrameSource);
		pDepthFrameSource->OpenReader(&_pFrameReader);
		pDepthFrameSource->get_DepthMaxReliableDistance(&_maxReliableDistance);
		pDepthFrameSource->get_DepthMinReliableDistance(&_minReliableDistance);
		CComPtr<IFrameDescription> pFrameDescription;
		pDepthFrameSource->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Height(&_height);
		pFrameDescription->get_Width(&_width);
	}

	HRESULT KinectDepthManager::acquireBuffer(std::vector<UINT16> &buffer) {
		CComPtr<IDepthFrame> depthFrame;
		HRESULT hResult = _pFrameReader->AcquireLatestFrame(&depthFrame);
		if (SUCCEEDED(hResult))
		{
			depthFrame->CopyFrameDataToArray(buffer.size(), &buffer[0]);
		}
		return hResult;
	}

	int KinectDepthManager::height() {
		return _height;
	}

	int KinectDepthManager::width() {
		return _width;
	}

	int KinectDepthManager::numPixels() {
		return _height*_width;
	}

	UINT16 KinectDepthManager::minReliableDistance() {
		return _minReliableDistance;
	}

	UINT16 KinectDepthManager::maxReliableDistance() {
		return _maxReliableDistance;
	}

	bool KinectDepthManager::isInsideView(int x, int y) {
		return (0 < x) && (x < _width) && (0 < y) && (y < _height);
	}

	bool KinectDepthManager::isReliable(int depth) {
		return (depth < _maxReliableDistance) && (depth > _minReliableDistance);
	}

	bool KinectDepthManager::isReliable(CameraSpacePoint position) {
		return isReliable(1000 * position.Z);
	}

	std::vector<UINT16> KinectDepthManager::makeBuffer(){
		return std::vector<UINT16>(numPixels());
	}
#pragma endregion

#pragma region Color Manager

	KinectColorManager::KinectColorManager() {
		CComPtr<IKinectSensor> pKinect;
		error_check(::GetDefaultKinectSensor(&pKinect));
		BOOLEAN isOpen;
		error_check(pKinect->get_IsOpen(&isOpen));
		if (!isOpen) {
			throw std::runtime_error("Kinect is not opened.");
		}

		CComPtr<IColorFrameSource> pFrameSource;
		pKinect->get_ColorFrameSource(&pFrameSource);
		pFrameSource->OpenReader(&_pReader);
		CComPtr<IFrameDescription> pFrameDescription;
		pFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &pFrameDescription);
		pFrameDescription->get_Height(&_height);
		pFrameDescription->get_Width(&_width);
		pFrameDescription->get_BytesPerPixel(&_bytesPerPixel);
	}

	HRESULT KinectColorManager::acquireBuffer(std::vector<BYTE> &buffer) {
		CComPtr<IColorFrame> colorFrame;
		HRESULT hResult = _pReader->AcquireLatestFrame(&colorFrame);
		if (SUCCEEDED(hResult))
		{
			colorFrame->CopyConvertedFrameDataToArray(buffer.size(), &buffer[0], ColorImageFormat::ColorImageFormat_Bgra);
		}
		return hResult;
	}

	int KinectColorManager::bufferSize() {
		return _height * _width * _bytesPerPixel;
	}

	int KinectColorManager::height() {
		return _height;
	}

	int KinectColorManager::width() {
		return _width;
	}

	bool KinectColorManager::isInsideView(int x, int y) {
		return (0 < x) && (x < _width) && (0 < y) && (y < _height);
	}

	std::vector<BYTE> KinectColorManager::makeBuffer() {
		return std::vector<BYTE>(bufferSize());
	}

#pragma endregion

#pragma region CoordinateManager
	KinectCoordManager::KinectCoordManager() {
		CComPtr<IKinectSensor> pKinect;
		error_check(::GetDefaultKinectSensor(&pKinect));
		pKinect->get_CoordinateMapper(&_pCoordMapper);
		CComPtr<IDepthFrameSource> pDepthFrameSource;
		pKinect->get_DepthFrameSource(&pDepthFrameSource);
		CComPtr<IFrameDescription> pDepthFrameDescription;
		pDepthFrameSource->get_FrameDescription(&pDepthFrameDescription);
		pDepthFrameDescription->get_Height(&_depthHeight);
		pDepthFrameDescription->get_Width(&_depthWidth);

		CComPtr<IColorFrameSource> pColorFrameSource;
		pKinect->get_ColorFrameSource(&pColorFrameSource);
		CComPtr<IFrameDescription> pFrameDescription;
		pColorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &pFrameDescription);
		pFrameDescription->get_Height(&_colorHeight);
		pFrameDescription->get_Width(&_colorWidth);	
	}

	UINT16 KinectCoordManager::getDepthAtColorPixel(int x_color, int y_color, const std::vector<UINT16> &depthBuffer) {
		
		std::vector<DepthSpacePoint> depthSpace(_colorWidth * _colorHeight);
		_pCoordMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthSpace.size(), &depthSpace[0]);

		int index = y_color * _colorWidth + x_color;
		int xD = (int)depthSpace[index].X;
		int yD = (int)depthSpace[index].Y;
		return depthBuffer[yD * _depthWidth + xD];
	}

	CameraSpacePoint KinectCoordManager::getPositionAtColorPixel(int x, int y, const std::vector<UINT16> &depthBuffer)
	{
		std::vector<CameraSpacePoint> cameraSpace(_colorWidth * _colorHeight);
		_pCoordMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], cameraSpace.size(), &cameraSpace[0]);
		int index = y * _colorWidth + x;
		return cameraSpace[index];
	}

	CameraSpacePoint KinectCoordManager::getPositionAtDepthPixel(int x, int y, const std::vector<UINT16> &depthBuffer) {
		DepthSpacePoint depthPoint{ x, y };
		int index = y * _depthWidth + x;
		UINT16 depth = depthBuffer[index];
		CameraSpacePoint position;
		_pCoordMapper->MapDepthPointToCameraSpace(depthPoint, depth, &position);
		return position;
	}

	void KinectCoordManager::projectPositionToDepthPixel(CameraSpacePoint position, DepthSpacePoint &depthPoint) {
		_pCoordMapper->MapCameraPointToDepthSpace(position, &depthPoint);
	}
#pragma endregion
}
