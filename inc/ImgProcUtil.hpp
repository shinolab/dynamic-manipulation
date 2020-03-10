#ifndef _IMG_PROC_UTIL_HPP
#define _IMG_PROC_UTIL_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace imgProc {
	class extractor {
	public:
		virtual ~extractor() {};
		virtual cv::Point2f center(const cv::Mat& img) = 0;
	};

	class threshold_extractor :public extractor {
	public:
		threshold_extractor(const cv::Scalar _lowerBound, const cv::Scalar _upperBound);
		cv::Point2f center(const cv::Mat& img) override;
	private:
		cv::Scalar _upperBound;
		cv::Scalar _lowerBound;
	};

	//determines the position of target object based on hue value of an image.
	//It accepts only RGB images.
	class hue_backproject_extractor :public extractor {
	public:
		hue_backproject_extractor(
			const std::vector<cv::Mat>& imgs_target,
			const int lowerBound,
			const int upperBound,
			int size_hist = 30);
		cv::Point2f center(const cv::Mat& img) override;
	private:
		cv::Mat _hist_target;
		int _lowerBound;
		int _upperBound;
	};

	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center);
	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center, cv::ColorConversionCodes code);
}

#endif // !_IMG_PROC_UTIL_HPP