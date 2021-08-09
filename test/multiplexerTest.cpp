#include <iostream>
#include <thread>
#include <chrono>
#include "WinMultiplexer.hpp"

std::pair<int, int> CreatePair(int a, int b) {
	return std::make_pair(10*a, 10*b);
}

int main(int argc, char** argv) {
	WinMultiplexer mux;
	auto p = CreatePair(1, 2);
	std::cout << "[TEST] " << p.first << "," << p.second << std::endl;
	mux.AddOrder([&p]() { std::cout << p.first << "," << p.second << std::endl; }, 2000);
	mux.AddOrder([]() { std::cout << "B" << std::endl; }, 1000);
	mux.Start();
	
	for (int i = 0; i < 100; i++) {
		std::cout << "Stopping MUX" << std::endl;
		mux.Stop();
		mux.ClearOrders();
		mux.AddOrder([]() { std::cout << "C" << std::endl; }, 10);
		mux.Start();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	mux.Stop();
	return 0;
}