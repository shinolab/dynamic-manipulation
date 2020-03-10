#include "ImgProcUtil.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace imgProc {
	threshold_extractor::threshold_extractor(
		const cv::Scalar lowerBound,
		cv::Scalar upperBound):
		_lowerBound(lowerBound),
		_upperBound(upperBound){}

	cv::Point2f threshold_extractor::extract_center(const cv::Mat& img) {
		cv::Mat img_bin;
		cv::inRange(img, _lowerBound, _upperBound, img_bin);
		cv::Moments mu = cv::moments(img_bin, true);
		return cv::Point2f(mu.m01 / mu.m00, mu.m10 / mu.m00);
	}

	hue_backproject_extractor::hue_backproject_extractor(
		const std::vector<cv::Mat>& img_target,
		int lowerBound,
		int upperBound,
		int size_hist):
		_lowerBound(lowerBound),
		_upperBound(upperBound) {
		std::vector<cv::Mat> imgs_target_hsv;
		for (auto itr = img_target.begin(); itr != img_target.end(); itr++) {
			cv::Mat img_target_hsv;
			cv::cvtColor(*itr, img_target_hsv, cv::COLOR_BGR2HSV);
			imgs_target_hsv.push_back(img_target_hsv);
		}
		std::vector<int> channels = { 0 };
		std::vector<int> sizes_hist = { 30 };
		std::vector<float> ranges_hist = { 0, 180 };
		cv::calcHist(imgs_target_hsv, channels, cv::Mat(), _hist_target, sizes_hist, ranges_hist);
		cv::normalize(_hist_target, _hist_target, 255, cv::NORM_MINMAX);
	}

	cv::Point2f hue_backproject_extractor::extract_center(const cv::Mat &img) {
		cv::Mat img_hsv;
		cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
		std::vector<cv::Mat> imgs_hsv = { img_hsv };
		cv::Mat backProjection;
		std::vector<int> channels = { 0 };
		std::vector<float> ranges = { 0, 180 };
		cv::calcBackProject(imgs_hsv, channels, _hist_target, backProjection, ranges, 1.0);
		cv::Mat extracted;
		cv::inRange(backProjection, _lowerBound, _upperBound, extracted);
		cv::Mat morph1;
		cv::morphologyEx(extracted,
			morph1,
			cv::MORPH_CLOSE,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)),
			cv::Point(-1, -1),
			1
		);
		cv::Mat morph2;
		cv::morphologyEx(morph1,
			morph2,
			cv::MORPH_OPEN,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)),
			cv::Point(-1, -1),
			1
		);
		cv::Moments mu = cv::moments(morph2, true);
		return cv::Point2f(mu.m01 / mu.m00, mu.m10 / mu.m00);
	}
}

void imgProc::get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center) {
	cv::Mat imgBin;
	cv::inRange(img, lowerbound, upperbound, imgBin);
	auto mu = cv::moments(imgBin, true);
	center = cv::Point2f((mu.m10 / mu.m00), (mu.m01 / mu.m00));
#ifdef _DEBUG
	cv::Mat imgDebug;
	cv::cvtColor(imgBin, imgDebug, cv::COLOR_GRAY2BGR);
	cv::circle(imgDebug, center, 3, cv::Scalar(0, 0, 255), -1);
	cv::imshow("center", imgDebug);
	cv::waitKey(5);
#endif
}

void imgProc::get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center, cv::ColorConversionCodes code) {
	cv::Mat img_to_process;
	cv::cvtColor(img, img_to_process, code);
	imgProc::get_center_thr(img_to_process, lowerbound, upperbound, center);
}
