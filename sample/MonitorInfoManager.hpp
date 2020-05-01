#ifndef _MONITOR_INFO_MANAGER_HPP
#define _MONITOR_INFO_MANAGER_HPP
#define NOMINMAX
#include <Windows.h>
#include <vector>

namespace MonitorInfoManager {
	void get_info(std::vector<MONITORINFOEX> &info_list);
	void get_main_monitor_info(int &width, int &height, int &pos_x, int &pos_y);
	void get_side_monitor_info(int &width, int &height, int &pos_x, int &pos_y);

};

#endif // !_MONITOR_INFO_MANAGER_HPP
