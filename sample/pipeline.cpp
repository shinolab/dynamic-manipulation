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

int main()
{
	HANDLE pipeHandle = CreateFileW(L"\\\\.\\pipe\\mynamedpipe", GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (pipeHandle == INVALID_HANDLE_VALUE)
	{
		std::cout << "Failed to create pipe." << std::endl;
		return -1;
	}
	std::cout << " Ready to Recieve Buffer. " << std::endl;
	char buffer[256];

	std::cout << "ODCS Initializing..." << std::endl;
	odcs odcs;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));

	odcs.StartControl(objPtrs);

	//write your application process here.

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
			v.push_back(atoi(temp.c_str()));
		}

		Eigen::Vector3f vec = Eigen::Map<Eigen::Vector3f>(&v[0]);
		objPtrs[0]->updateStatesTarget(vec, Eigen::Vector3f(0, 0, 0));
		std::cout << "target position is updated to : " << vec.transpose() << std::endl;

	} while (strcmp(buffer, "") != 0);

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
