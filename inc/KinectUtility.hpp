#ifndef _KINECT_UTILITY_HPP
#define _KINECT_UTILITY_HPP

#include "Kinect.h"
#include <atlbase.h>
#include <sstream>
#include <vector>

namespace KinectUtility{

	inline void error_check(HRESULT hr) noexcept(false) {	
		if (hr != S_OK) {
			std::stringstream ss;
			ss << "failed " << std::hex << hr << std::endl;
			throw std::runtime_error(ss.str().c_str());
		}
	}

	class KinectDepthManager {
	public:
		KinectDepthManager();
		HRESULT acquireBuffer(std::vector<UINT16> &buffer);
		int numPixels();
		int height();
		int width();
		UINT16 minReliableDistance();
		UINT16 maxReliableDistance();
		bool isInsideView(int x, int y);
		bool isReliable(CameraSpacePoint csp);
		bool isReliable(int depth);
		std::vector<UINT16> makeBuffer(); //make storage of the size appropriate to store a Kinect V2 depth image
		
	private:
		CComPtr<IDepthFrameReader> _pFrameReader;
		int _height;
		int _width;
		UINT16 _maxReliableDistance;
		UINT16 _minReliableDistance;
	};

	class KinectColorManager {
	public:
		KinectColorManager();
		HRESULT acquireBuffer(std::vector<BYTE> &buffer);
		int height();
		int width();
		int bufferSize();
		bool isInsideView(int x, int y);
		std::vector<BYTE> makeBuffer(); //create buffer of the size appropriate for storing a Kinect V2 RGBA image.
	private:
		CComPtr<IColorFrameReader> _pReader;
		int _height;
		int _width;
		unsigned int _bytesPerPixel;
	};

	/*
	provide helper functions for coordinate mapping between color/depth/camera space
	of a specified Kinect hardware
	*/
	class KinectCoordManager {
	public:
		KinectCoordManager();
		UINT16 getDepthAtColorPixel(int x_color, int y_color, const std::vector<UINT16> &depthBuffer);
		CameraSpacePoint getPositionAtColorPixel(int x_color, int y_color, const std::vector<UINT16> &depthBuffer);
		CameraSpacePoint getPositionAtDepthPixel(int x_depth, int y_depth, const std::vector<UINT16> &depthBuffer);
		void projectPositionToDepthPixel(CameraSpacePoint position, DepthSpacePoint &depthPoint);
	private:
		CComPtr<ICoordinateMapper> _pCoordMapper;
		int _colorWidth;
		int _colorHeight;
		int _depthWidth;
		int _depthHeight;
	};
}


#endif