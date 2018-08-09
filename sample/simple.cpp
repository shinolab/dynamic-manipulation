#include "odcs.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>

#pragma comment(lib, "winmm.lib")

int main()
{
	std::ofstream ofs("20180802_balloon_depth_log.csv");
	ofs << "#, x, y, z" << std::endl;
	//odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	//odcs.Initialize();

	KinectApp app;
	app.initialize();
	Sleep(1000);
	app.getDepthBuffer();
	for (int i = 0; i < 1000; i++)
	{
		HRESULT hr = app.getDepthBuffer();
		if (SUCCEEDED(hr))
		{
			auto csp = app.getPositionAtDepthPixel(244, 187);
			ofs << i << ", " << csp.X << ", " << csp.Y << ", " << csp.Z << std::endl;
		}
		Sleep(50);
	}
	ofs.close();
	return 0;
	//std::vector<FloatingObjectPtr> objPtrs;
	//objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485))); // add object
	//odcs.StartControl(objPtrs);

	std::cout << "Press any key to close." << std::endl;
	getchar();
	//odcs.Close();
	return 0;
}
