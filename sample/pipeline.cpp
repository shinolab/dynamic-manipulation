#include "odcs.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

//グローバル座標をUnity座標に変換する。
Eigen::Vector3f transformGlobal2Unity(Eigen::Vector3f const &r)
{
	Eigen::Vector3f r1
		= Eigen::AngleAxisf(130.5 * M_PI / 180.0, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX())
		* Eigen::Translation3f(Eigen::Vector3f(-82, 385, 93))*r;

	return 0.1 * Eigen::Vector3f(r1.x() + 15, -r1.y(), r1.z() + 15);
}
//Unity座標をグローバル座標に変換する.mainではこれが回っている。
Eigen::Vector3f transformUnity2Global(Eigen::Vector3f const &r)
{
	Eigen::Matrix3f rotation;
	rotation << -1,0,0,
		0,0,-1,
		0,1,0;
	return (rotation * r + Eigen::Vector3f(610, 800, 95));
	//return (r.transpose() * rotation + Eigen::Vector3f(610,570,-55).transpose()).transpose();

}

int main()
{
	std::cout << "ODCS Initializing..." << std::endl;
	odcs odcs;
	odcs.Initialize();
	odcs.AddObject(Eigen::Vector3f(0, 0, 1350));
	odcs.StartControl();
	getchar();
	//write your application process here.
	HANDLE pipeHandle = CreateFileW(L"\\\\.\\pipe\\mynamedpipe", GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (pipeHandle == INVALID_HANDLE_VALUE)
	{
		std::cout << "Failed to create pipe." << std::endl;
		return -1;
	}
	std::cout << " Ready to Recieve Buffer. " << std::endl;
	char buffer[256];

	do
	{
		buffer[0] = '\0';
		DWORD readBytes;
		ReadFile(pipeHandle, buffer, sizeof(buffer), &readBytes, NULL);
		if (buffer[0] == '\0')
		{
			continue;
		}
		buffer[readBytes] = '\0';
		std::string str(buffer);
		std::istringstream ss(str);
		std::vector<float> v;
		std::string temp;
		while (std::getline(ss, temp, ','))
		{
			v.push_back(atof(temp.c_str()));
		}
		Eigen::Vector3f pos = Eigen::Map<Eigen::Vector3f>(&v[0]);
		odcs.GetFloatingObject(0)->updateStatesTarget(transformUnity2Global(pos), Eigen::Vector3f(0, 0, 0));
		std::cout << "target position is updated to : " << odcs.GetFloatingObject(0)->getPositionTarget().transpose() << std::endl;

	} while (strcmp(buffer, "") != 0);

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
