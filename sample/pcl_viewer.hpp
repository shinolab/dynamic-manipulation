#ifndef _DYNAMAN_PCL_VIEWER_HPP
#define _DYNAMAN_PCL_VIEWER_HPP

#include <vector>
#include <string>
#include <functional>
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct float3 {
	float x, y, z;
	float3 operator+(const float3& another) {
		return float3{ x + another.x, y + another.y, z + another.z };
	}

	float3& operator=(const float3& another) {
		x = another.x;
		y = another.y;
		z = another.z;
		return *this;
	}
};

class window {
public:
	std::function<void(bool)> on_left_mouse = [](bool) {};
	std::function<void(bool)> on_right_mouse = [](bool) {};
	std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
	std::function<void(double, double)> on_mouse_move = [](double, double) {};
	std::function<void(int)> on_key_release = [](int) {};
	window(const std::string& title, int width, int height);
	~window();
	void close();
	int width() const;
	int height() const;
	operator bool();
private:
	GLFWwindow* m_win;
	int m_width;
	int m_height;
	std::string m_title;
};


class pcl_viewer {
	using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
	struct State {
		float offset_x, offset_y, last_x, last_y, pitch, yaw;
		bool ml, mr;
	};
public:
	pcl_viewer(const std::string& title, int width, int height);
	~pcl_viewer();
	void draw(const std::vector<pcl_ptr>& clouds, const std::vector<float3>& colors);
	void draw(const std::vector<pcl_ptr>& clouds);
	operator bool();

private:
	void initialize_callbacks();
	std::vector<float3> pcl_viewer::make_default_colors(int size);
	State m_state;
	window m_win;
};

#endif // !_DYNAMAN_PCL_VIEWER_HPP
