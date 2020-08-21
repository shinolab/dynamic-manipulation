#include "pcl_viewer.hpp"

window::window(const std::string& title, int width, int height)
:m_title(title), m_width(width), m_height(height) 
{
	glfwInit();
	m_win = glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
	if (!m_win) {
		throw std::runtime_error("Failed to create a window. Please check your graphic driver.");
	}
	glfwMakeContextCurrent(m_win);
	glfwSetWindowUserPointer(m_win, this);
	glfwSetMouseButtonCallback(m_win, [](GLFWwindow* w, int button, int action, int mods)
		{
			auto s = static_cast<window*>(glfwGetWindowUserPointer(w));
			if (button == 0)
				s->on_left_mouse(action == GLFW_PRESS);
			if (button == 1)
				s->on_right_mouse(action == GLFW_PRESS);
		}
	);
	glfwSetScrollCallback(m_win, [](GLFWwindow* w, double xoffset, double yoffset)
		{
			auto s = static_cast<window*>(glfwGetWindowUserPointer(w));
			s->on_mouse_scroll(xoffset, yoffset);
		}
	);
	glfwSetCursorPosCallback(m_win, [](GLFWwindow* w, double x, double y)
		{
			auto s = static_cast<window*>(glfwGetWindowUserPointer(w));
			s->on_mouse_move(x, y);
		}
	);
	glfwSetKeyCallback(m_win, [](GLFWwindow* w, int key, int scancode, int action, int mods)
		{
			auto s = static_cast<window*>(glfwGetWindowUserPointer(w));
			if (0 == action)
				s->on_key_release(key);
		}
	);
}

window::~window() {
	glfwDestroyWindow(m_win);
	glfwTerminate();
}

int window::width() const {
	return m_width;
}

int window::height() const{
	return m_height;
}

void window::close() {
	glfwSetWindowShouldClose(m_win, 1);
}

window::operator bool() {

	glPopMatrix();
	glfwSwapBuffers(m_win);

	auto res = !glfwWindowShouldClose(m_win);

	glfwPollEvents();
	glfwGetFramebufferSize(m_win, &m_width, &m_height);

	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, m_width, m_height);

	glPushMatrix();
	glfwGetWindowSize(m_win, &m_width, &m_height);
	glOrtho(0, m_width, m_height, 0, -1, +1);

	return res;
}

void pcl_viewer::initialize_callbacks() {
	m_win.on_left_mouse = [&](bool pressed)
	{
		m_state.ml = pressed;
	};

	m_win.on_right_mouse = [&](bool pressed) {
		m_state.mr = pressed;
		m_state.offset_x += 0.1f;
	};

	m_win.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		m_state.offset_x += static_cast<float>(xoffset);
		m_state.offset_y += static_cast<float>(yoffset);
	};

	m_win.on_mouse_move = [&](double x, double y)
	{
		if (m_state.ml)
		{
			m_state.yaw -= (x - m_state.last_x);
			m_state.pitch += (y - m_state.last_y);
		}
		m_state.last_x = x;
		m_state.last_y = y;
	};

	m_win.on_key_release = [&](int key)
	{
		if (key == 32) // Escape
		{
			m_state.yaw = m_state.pitch = 0; m_state.offset_x = m_state.offset_y = 0.0;
		}
	};
}

pcl_viewer::pcl_viewer(const std::string& title, int width, int height)
	:m_win(title, width, height), m_state{ 0, 0, 0, 0, 0, false, false }
{
	initialize_callbacks();
}

pcl_viewer::~pcl_viewer() {}

void pcl_viewer::draw(const std::vector<pcl_ptr>& clouds, const std::vector<float3>& colors) {
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glClearColor(0.f, 0.f, 0.f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60.0, static_cast<double>(m_win.width()) / m_win.height(), 0.1f, 10.f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(
		0, 0, 0,
		0, 0, 1,
		1, 0, 0
	);
	glTranslatef(0 + m_state.offset_x, 0, 0.5f + m_state.offset_y * 0.05f);
	glRotated(-M_PI_2 + m_state.pitch, 0, 1, 0);
	glRotated(m_state.yaw, 0, 0, 1);
	glTranslatef(0, 0, 0);
	glPointSize(2.0f);
	glEnable(GL_TEXTURE_2D);

	int color_idx = 0;
	for (auto&& cloud : clouds)
	{
		auto color = colors[(color_idx++) % colors.size()];
		glBegin(GL_POINTS);
		glColor3f(color.x, color.y, color.z);

		for (auto&& point : cloud->points) {
			if (point.z) {
				glVertex3f(point.x, point.y, point.z);
			}
		}
		glEnd();
	}

	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(0.1f, 0.f, 0.f);
	glEnd();
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(0.0f, 0.1f, 0.0f);
	glEnd();
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(0.f, 0.f, 0.1f);
	glEnd();

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}


void pcl_viewer::draw(const std::vector<pcl_ptr>& clouds) {
	draw(clouds, make_default_colors(clouds.size()));
}

std::vector<float3> pcl_viewer::make_default_colors(int size) {
	return std::vector<float3>{
		{ 0.9f, 0.1f, 0.1f },
		{ 0.1f, 0.9f, 0.1f },
		{ 0.1f, 0.1f, 0.9f }
	};
}

pcl_viewer::operator bool() {
	return m_win;
}