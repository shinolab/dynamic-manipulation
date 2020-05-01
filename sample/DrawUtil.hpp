#ifndef _DRAW_UTILITY_HPP
#define _DRAW_UTILITY_HPP
#include <opencv2/core.hpp>
#include <vector>

namespace DrawUtil {
	std::vector<cv::Point2f> drawCircleGrid(cv::Mat &dst, cv::Point upperLeftCorner, cv::Size patternSize, int gridStepX, int gridStepY, int radius = -1);

	void imshowPopUp(const char* windowName, cv::Mat image, int posX, int posY);
}

#endif // !_DRAW_UTILITY_HPP
