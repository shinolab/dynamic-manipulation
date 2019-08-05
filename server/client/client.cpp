#include "client.hpp"
#include "query.hpp"
#include "StructConverter.hpp"
#include <memory>
#include <iostream>

client::client(const char* ipAddr, const int port) {
	WSAStartup(MAKEWORD(2, 0), &data);
	//std::cout << "Please enter the IP Address of the destination: ";
	//std::cin >> destination;

	memset(&dstAddr, 0, sizeof(dstAddr));
	dstAddr.sin_port = htons(port);
	dstAddr.sin_family = AF_INET;
	dstAddr.sin_addr.s_addr = inet_addr(ipAddr);
	std::cout << "Connecting to " << ipAddr << "..." << std::endl;

	dstSocket = socket(AF_INET, SOCK_STREAM, 0);

	if (connect(dstSocket, (sockaddr*)&dstAddr, sizeof(dstAddr))) {
		std::cout << "Connection Failed." << std::endl;
	}
	std::cout << "Connected to " << ipAddr << std::endl;
}

client::~client() {
	closesocket(dstSocket);
	WSACleanup();
}

void client::sendQuery(orderID id) {
	auto q = std::unique_ptr<query>(new query{id});
	StructConverter::Struct2Buffer(q.get(), &_queryBuffer[0]);
	send(dstSocket, _queryBuffer, BUFFER_SIZE, 0);
	recv(dstSocket, _replyBuffer, BUFFER_SIZE, 0);
}

const char* client::replyBuffer() {
	return &_replyBuffer[0];
}

void client::queryPosition(position* positionPtr) {
	sendQuery(orderID::POSITION);
	StructConverter::Buffer2Struct(_replyBuffer, positionPtr);
}