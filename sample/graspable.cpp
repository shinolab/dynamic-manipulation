#include <vector>
#include <memory>
#include <thread>
#include <iostream>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include "stb_easy_font.h"
#include "example.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "StereoTracker.hpp"
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points) {
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto pVertex = points.get_vertices();
	for (auto& p : cloud->points) {
		p.x = pVertex->x;
		p.y = pVertex->y;
		p.z = pVertex->z;
		pVertex++;
	}
	return cloud;
}

pcl_ptr points_to_pcl(const std::vector<Eigen::Vector3f>& points) {
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = points.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	for (int iv = 0; iv < points.size(); iv++) {
		cloud->points[iv].x = points[iv].x();
		cloud->points[iv].y = points[iv].y();
		cloud->points[iv].z = points[iv].z();
	}
	return cloud;
}

namespace {
	// Struct for managing rotation of pointcloud view
	struct state {
		state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
			ml(false), offset_x(0.0f), offset_y(0.0f) {}
		double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
	};

	float3 colors[]{ { 0.8f, 0.1f, 0.3f },
					  { 0.1f, 0.9f, 0.5f },
	};
}

void register_glfw_callbacks(window& app, state& app_state) {
	app.on_left_mouse = [&](bool pressed)
	{
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		app_state.offset_x += static_cast<float>(xoffset);
		app_state.offset_y += static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y)
	{
		if (app_state.ml)
		{
			app_state.yaw -= (x - app_state.last_x);
			app_state.yaw = std::max(app_state.yaw, -120.0);
			app_state.yaw = std::min(app_state.yaw, +120.0);
			app_state.pitch += (y - app_state.last_y);
			app_state.pitch = std::max(app_state.pitch, -80.0);
			app_state.pitch = std::min(app_state.pitch, +80.0);
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key)
	{
		if (key == 32) // Escape
		{
			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
		}
	};
}

void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points) {
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points)
	{
		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

		glBegin(GL_POINTS);
		glColor3f(c.x, c.y, c.z);

		/* this segment actually prints the pointcloud */
		for (int i = 0; i < pc->points.size(); i++)
		{
			auto&& p = pc->points[i];
			if (p.z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3f(p.x, p.y, p.z);
			}
		}
		glEnd();
	}
	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

int main(int argc, char** argv) {

	std::string target_image_name("blue_target_no_cover.png");
	auto tracker = haptic_icon::CreateTracker(target_image_name);

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen()) {
		return ENXIO;
	}
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
	);
	
	std::thread t_viewer(
		[&pObject]() {
			window viewer(1280, 720, "pointcloud");
			state viewer_state;
			register_glfw_callbacks(viewer, viewer_state);

			rs2::pipeline pipe;
			rs2::config cfg;
			cfg.enable_stream(RS2_STREAM_DEPTH);
			pipe.start(cfg);
			Eigen::Matrix3f rot_rs;
			rot_rs <<
				-0.698625, -0.0134938, 0.715361,
				-0.71529, -0.0103563, -0.698751,
				0.0168372, -0.999855, -0.00241686;
			Eigen::Vector3f pos_rs(-409.233, 460.217, -7.72512);
			Eigen::Affine3f affine_rs = Eigen::Affine3f::Identity();
			affine_rs.rotate(rot_rs);
			affine_rs.translate(pos_rs);
			while (viewer) {
				rs2::pointcloud points_rs;
				auto balloon_point = points_to_pcl(std::vector<Eigen::Vector3f>{pObject->getPosition()});
				auto cloud = points_to_pcl(points_rs.calculate(pipe.wait_for_frames().get_depth_frame()));
				pcl_ptr cloud_global;
				pcl::transformPointCloud(*cloud, *cloud_global, affine_rs);

				std::vector<pcl_ptr> cloud_ptrs{ cloud, balloon_point };
				draw_pointcloud(viewer, viewer_state, cloud_ptrs);
			}
		}
	);

	t_viewer.join();
	return 0;

}
