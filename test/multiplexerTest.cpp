#include <iostream>
#include <thread>
#include <chrono>
#include "WinMultiplexer.hpp"

int main(int argc, char** argv) {
	WinMultiplexer mux;
	mux.AddOrder([]() { std::cout << "A" << std::endl; }, 200000);
	mux.AddOrder([]() { std::cout << "B" << std::endl; }, 1000000);
	mux.Start();

	std::this_thread::sleep_for(std::chrono::seconds(10));
	std::cout << "Stopping MUX" << std::endl;
	mux.Stop();
	mux.ClearOrders();
	mux.AddOrder([]() { std::cout << "C" << std::endl; }, 100000);
	mux.Start();
	std::this_thread::sleep_for(std::chrono::seconds(10));
	mux.Stop();
	return 0;
}