#include "winMultiplexer.hpp"
#include <Windows.h>
#include <vector>
#include <utility>
#include <functional>
#include <shared_mutex>

#pragma comment(lib, "winmm")

winMultiplexer::winMultiplexer(std::vector<std::pair<std::function<void()>, double>> &orders_init) : orders(orders_init){
	QueryPerformanceFrequency(&freq);
}

winMultiplexer::~winMultiplexer() {
	if(isRunning())
		Stop();
}

void winMultiplexer::Run() {
	std::shared_lock<std::shared_mutex> lk(mtx_order);
	for (auto itr = orders.begin(); itr != orders.end(); itr++) {
		LARGE_INTEGER start, elapsed;
		(*itr).first();
		QueryPerformanceCounter(&start);
		do {
			Sleep(0);
			QueryPerformanceCounter(&elapsed);
		} while (1000 * (double)(elapsed.QuadPart - start.QuadPart) / (double)freq.QuadPart < (*itr).second);
	}
}

void winMultiplexer::RunAsync() {
	flagRunning = true;
	th_exe = std::thread([this]() {
		while (isRunning()) {
			Run();
		}
	});
}

bool winMultiplexer::isRunning() {
	std::shared_lock<std::shared_mutex> lk(mtx_flagRunning);
	return flagRunning;
}

void winMultiplexer::Stop() {
	{
		std::lock_guard<std::shared_mutex> lk(mtx_flagRunning);
		flagRunning = false;
	}
	if (th_exe.joinable()) {
		th_exe.join();
	}
}

void winMultiplexer::SetOrders(std::vector<std::pair<std::function<void()>, double>> &ordersNew) {
	Stop();
	orders = ordersNew;
	RunAsync();
}


