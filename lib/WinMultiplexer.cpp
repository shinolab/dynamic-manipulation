#include "WinMultiplexer.hpp"
#include <Windows.h>
#include <vector>
#include <utility>
#include <functional>
#include <shared_mutex>
#include <iostream>

#pragma comment(lib, "winmm")

WinMultiplexer::WinMultiplexer()
{
	QueryPerformanceFrequency(&freq);
}

WinMultiplexer::~WinMultiplexer() {
	if(IsRunning())
		Stop();
}


void WinMultiplexer::MicroSleep(int interval_us) 
{
	LARGE_INTEGER start, now;
	QueryPerformanceCounter(&start);
	const auto sleep_count = interval_us * (freq.QuadPart / 1000000L);
	do {
		QueryPerformanceCounter(&now);
	} while (now.QuadPart - start.QuadPart  < sleep_count);
}

void WinMultiplexer::Start() {
	this->Stop();
	m_is_running = true;
	th_exe = std::thread([this]() {
		auto itr = orders.begin();
		while (IsRunning()) {
			LARGE_INTEGER start, now;
			QueryPerformanceCounter(&start);
			itr->first();
			itr++;
			if (itr == orders.end()) {
				itr = orders.begin();
			}
			const auto sleep_count = itr->second * (freq.QuadPart / 1000000L);
			do {
				QueryPerformanceCounter(&now);
			} while (now.QuadPart - start.QuadPart < sleep_count);
		}
	});
}

bool WinMultiplexer::IsRunning() {
	std::shared_lock<std::shared_mutex> lk(mtx_is_running);
	return m_is_running;
}

void WinMultiplexer::Stop() {
	{
		std::lock_guard<std::shared_mutex> lk(mtx_is_running);
		m_is_running = false;
	}
	if (th_exe.joinable()) {
		std::cout << "Joining th_exe.";
		th_exe.join();
		std::cout << ".... Joined." << std::endl;
	}
}

void WinMultiplexer::AddOrder(std::function<void()> order, int interval_us) {
	orders.push_back(std::make_pair(order, interval_us));
}

void WinMultiplexer::ClearOrders() {
	orders.clear();
}

