#ifndef _IMG_PROC_UTIL_HPP
#define _IMG_PROC_UTIL_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace imgProcUtil {
	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center);
	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center, cv::ColorConversionCodes code);
}

#endif // !_IMG_PROC_UTIL_HPP