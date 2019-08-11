#include "server.hpp"
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <functional>
#include <iostream>

#pragma comment(lib, "ws2_32")

server::server(const std::function<void (const char*, char*)> &process, const int port) {
	dstAddrSize = sizeof(dstAddr);
	WSAStartup(MAKEWORD(2, 0), &data);
	srcAddr.sin_port = htons(port);
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	srcSocket = socket(AF_INET, SOCK_STREAM, 0);
	bind(srcSocket, (sockaddr*)&srcAddr, sizeof(srcAddr));
	_process = process;
}

server::~server() {
	WSACleanup();
}

void server::start() {
	listen(srcSocket, 1);
	std::cout << "Waiting for connection..." << std::endl;
	dstSocket = accept(srcSocket, (sockaddr*)&dstAddr, &dstAddrSize);
	while (1) {
		int numrcv = recv(dstSocket, queryBuffer, sizeof(char) * BUFFER_SIZE, 0);
		if (numrcv == 0 || numrcv == -1) {
			status = closesocket(dstSocket);
			break;
		}
		_process(queryBuffer, replyBuffer);
		send(dstSocket, replyBuffer, sizeof(char) * BUFFER_SIZE, 0);
	}
	
}
