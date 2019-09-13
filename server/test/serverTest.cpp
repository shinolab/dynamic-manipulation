#include "server.hpp"
#include "odcs.hpp"
#include "position.hpp"
#include "query.hpp"
#include "StructConverter.hpp"
#include "../sample/init_uist.hpp"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <iostream>

using namespace dynaman;

int main() {
	//sensor();
	//
	Eigen::Vector3f posTgt(-50.0f, 496.04f, 1231.f);
	FloatingObjectPtr objPtr = FloatingObject::Create(posTgt, -0.0001f, 127.f);
	odcs dynaman;
	initialize_uist_setup(dynaman);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();
	const int port = 2001;
	auto srvPtr = std::unique_ptr<server>(new server(
		[&objPtr](const char* queryBuffer, char* replyBuffer) {
		Eigen::Vector3f pos = objPtr->getPosition();
		//position data{1.0f, 2.0f, 3.0f};//for test
		position data{ pos.x(), pos.y(), pos.z() };
		StructConverter::Struct2Buffer<position>(&data, replyBuffer);
	}, port));

	srvPtr->start();
	dynaman.Close();
	return 0;
}