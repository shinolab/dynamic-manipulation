#ifndef _WIN_MULTIPLEXER_HPP
#define _WIN_MULTIPLEXER_HPP

#include <Windows.h>
#include <vector>
#include <utility>
#include <functional>
#include <thread>
#include <shared_mutex>

class WinMultiplexer {
private:
	std::vector<std::pair<std::function<void()>, int>> orders;
	std::function<void()> onStop;
	std::shared_mutex mtx_is_running;
	LARGE_INTEGER freq;
	bool m_is_running = false;
	std::thread th_exe;
public:
	WinMultiplexer();
	~WinMultiplexer();
	void Start();
	void Stop();
	void MicroSleep(int interval_us);
	void AddOrder(std::function<void()> order, int interval_us);
	void SetStopSequence(std::function<void()> sequence);
	void ClearOrders();
	bool IsRunning();
};

#endif

