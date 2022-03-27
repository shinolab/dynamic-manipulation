#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace imgProc {
	class extractor {
	public:
		virtual ~extractor() {};
		virtual cv::Point2f extract_center(const cv::Mat& img) = 0;
		virtual cv::Mat img_debug() = 0;
	};

	class threshold_extractor :public extractor {
	public:
		threshold_extractor(cv::Scalar lowerBound, cv::Scalar upperBound);
		static std::shared_ptr<threshold_extractor> create(cv::Scalar lowerBound, cv::Scalar upperBound);
		cv::Point2f extract_center(const cv::Mat& img) override;
		cv::Mat img_debug() override;
	private:
		cv::Scalar _upperBound;
		cv::Scalar _lowerBound;
		cv::Mat _img_processed;
	};

	//determines the position of target object based on hue value of an image.
	//It accepts only RGB images.
	class hue_backproject_extractor :public extractor {
	public:
		hue_backproject_extractor(
			const std::vector<cv::Mat>& imgs_target,
			int lowerBound,
			int upperBound,
			int size_hist = 30
		);

		static std::shared_ptr<hue_backproject_extractor> create(
			const std::vector<cv::Mat>& imgs_target,
			int lowerBound = 10,
			int upperBound = 255,
			int size_hist = 30
		);

		static std::shared_ptr<hue_backproject_extractor> create(
			const std::string& img_target_name,
			int lowerBound = 10,
			int upperBound = 255,
			int sizeHist = 30
		);

		cv::Point2f extract_center(const cv::Mat& img) override;

		cv::Mat img_debug() override;
	private:
		cv::Mat _hist_target;
		cv::Mat _img_result;
		cv::Mat _img_backProject;
		int _lowerBound;
		int _upperBound;
	};

	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center);
	void get_center_thr(const cv::Mat& img, const cv::Scalar lowerbound, const cv::Scalar upperbound, cv::Point2f& center, cv::ColorConversionCodes code);
}
