#include "MonitorInfoManager.hpp"
#include <Windows.h>
#include <algorithm>
#include <vector>

namespace{
	static std::vector<MONITORINFOEX> info_list_internal;
	BOOL monitorEnumProc(HMONITOR hMonitor, HDC hdc, LPRECT lprect, LPARAM lparam) {
		MONITORINFOEX info;
		info.cbSize = sizeof(info);
		GetMonitorInfo(hMonitor, &info);
		info_list_internal.push_back(info);
		return TRUE;
	}
};

//internally used to retrieve monitor data

void MonitorInfoManager::get_info(std::vector<MONITORINFOEX> &info_list) {
	info_list_internal.clear();
	EnumDisplayMonitors(NULL, NULL, monitorEnumProc, 0);
	info_list = info_list_internal;
}

void MonitorInfoManager::get_main_monitor_info(int &width, int &height, int &pos_x, int &pos_y) {
	std::vector<MONITORINFOEX> monitor_info;
	MonitorInfoManager::get_info(monitor_info);
	auto main_info = *std::find_if(monitor_info.begin(), monitor_info.end(), [](MONITORINFOEX info) {return info.dwFlags == 1; });
	width = main_info.rcMonitor.right - main_info.rcMonitor.left;
	height = main_info.rcMonitor.bottom - main_info.rcMonitor.top;
	pos_x = main_info.rcMonitor.left;
	pos_y = main_info.rcMonitor.top;

}

void MonitorInfoManager::get_side_monitor_info(int &width, int &height, int &pos_x, int &pos_y) {
	std::vector<MONITORINFOEX> monitor_info;
	MonitorInfoManager::get_info(monitor_info);
	auto itr_monitor = std::find_if(monitor_info.begin(), monitor_info.end(), [](MONITORINFOEX info) {return info.dwFlags != 1; });
	if (itr_monitor != monitor_info.end()) {
		width = (*itr_monitor).rcMonitor.right - (*itr_monitor).rcMonitor.left;
		height = (*itr_monitor).rcMonitor.bottom - (*itr_monitor).rcMonitor.top;
		pos_x = (*itr_monitor).rcMonitor.left;
		pos_y = (*itr_monitor).rcMonitor.top;
	}
}