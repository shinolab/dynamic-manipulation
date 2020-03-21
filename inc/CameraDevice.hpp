#ifndef _CAMERA_DEVICE_HPP
#define _CAMERA_DEVICE_HPP

#include "xiApiPlusOcv.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <memory>

class CameraDevice {
public:
	virtual ~CameraDevice();
	virtual void open() = 0;
	virtual void close() = 0;
	virtual void fetch_frame(cv::Mat& img) = 0;
	void setIntrinsic(const cv::Mat& intrinsic);
	void getIntrinsic(cv::Mat& intrinsic) const;
	void setDistCoeff(const cv::Mat& distCoeff);
	void getDistCoeff(cv::Mat& distCoeff) const;
	virtual std::string id() = 0;

private:
	cv::Mat _intrinsic;
	cv::Mat _distCoeff;
};

class ximeaCameraDevice : public CameraDevice {
public:
	ximeaCameraDevice(const std::string& cam_id);
	~ximeaCameraDevice();
	static std::shared_ptr<ximeaCameraDevice> create(const std::string& cam_id);
	void open() override;
	void close() override;
	void fetch_frame(cv::Mat& img) override;
	std::string id() override;
private:
	xiAPIplusCameraOcv _cam;
	std::string _cam_id;
};

class photoDevice : public CameraDevice {
public:
	photoDevice(const std::string &filename);
	~photoDevice();
	static std::shared_ptr<photoDevice> create(const std::string &filename);
	void open() override;
	void close() override;
	void fetch_frame(cv::Mat& img) override;
	std::string id() override;
private:
	std::string _filename;
	cv::Mat _img;
};

#endif // !_CAMERA_DEVICE_HPP
