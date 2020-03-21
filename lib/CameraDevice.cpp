#include "CameraDevice.hpp"
#include "xiApiPlusOcv.hpp"
#include <string>
#include <memory>


CameraDevice::~CameraDevice() {};

void CameraDevice::getIntrinsic(cv::Mat &intrinsic) const {
	this->_intrinsic.copyTo(intrinsic);
}

void CameraDevice::setIntrinsic(const cv::Mat &intrinsic) {
	intrinsic.copyTo(this->_intrinsic);
}

void CameraDevice::getDistCoeff(cv::Mat& distCoeff) const {
	this->_distCoeff.copyTo(distCoeff);
}

void CameraDevice::setDistCoeff(const cv::Mat& distCoeff) {
	distCoeff.copyTo(this->_distCoeff);
}

ximeaCameraDevice::ximeaCameraDevice(const std::string& cam_id):_cam_id(cam_id) {}

std::shared_ptr<ximeaCameraDevice> ximeaCameraDevice::create(const std::string& cam_id) {
	return std::make_shared<ximeaCameraDevice>(cam_id);
}

ximeaCameraDevice::~ximeaCameraDevice() {}

void ximeaCameraDevice::open() {
	_cam.OpenBySN(&_cam_id[0]);
	_cam.SetExposureTime(10000);
	_cam.StartAcquisition();
	_cam.SetImageDataFormat(XI_IMG_FORMAT::XI_RGB24);
	//_cam.EnableWhiteBalanceAuto();
}

void ximeaCameraDevice::close() {
	_cam.Close();
}

std::string ximeaCameraDevice::id() {
	return _cam_id;
}

void ximeaCameraDevice::fetch_frame(cv::Mat& img) {
	img = _cam.GetNextImageOcvMat();
}

photoDevice::photoDevice(const std::string& filename){
	_filename = filename;
}

photoDevice::~photoDevice(){}

void photoDevice::open() {
	_img = cv::imread(_filename);
}

void photoDevice::close() {}

std::shared_ptr<photoDevice> photoDevice::create(const std::string& filename) {
	return std::make_shared<photoDevice>(filename);
}

void photoDevice::fetch_frame(cv::Mat& img) {
	_img.copyTo(img);
}

std::string photoDevice::id() {
	return _filename;
}