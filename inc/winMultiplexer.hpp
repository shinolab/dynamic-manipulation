#include <Windows.h>
#include <vector>
#include <utility>
#include <functional>
#include <thread>
#include <shared_mutex>

#pragma comment(lib, "winmm")

//execution interval must be specified in [milliseconds]. Windows Vista or newer is required.
class winMultiplexer {
public:
	typedef std::pair<std::function<void()>, double> unit_order_type;
	typedef std::vector<std::pair<std::function<void()>, double>> orders_vector;
private:
	std::vector<std::pair<std::function<void()>, double>> &orders;
	std::shared_mutex mtx_order;
	std::shared_mutex mtx_flagRunning;
	LARGE_INTEGER freq;
	bool flagRunning = false;
	std::thread th_exe;
public:
	winMultiplexer(std::vector<std::pair<std::function<void()>, double>> &orders);
	~winMultiplexer();
	void Run();
	void RunAsync();
	void Stop();
	void SetOrders(std::vector<std::pair<std::function<void()>, double>> &orders);
	bool isRunning();
};