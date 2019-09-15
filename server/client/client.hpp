#ifndef _TCP_IP_CLIENT_HPP
#define _TCP_IP_CLIENT_HPP
#include "query.hpp"
#include "position.hpp"
#include <WinSock2.h>
#include <WS2tcpip.h>

#pragma comment(lib, "ws2_32")

class client {
public:
	client(const char* ipAddr, const int port);
	~client();
	void sendQuery(orderID id);
	const char* replyBuffer();
	void queryPosition(position* positionPtr);
private:
	enum { BUFFER_SIZE = 1024 };
	int dstSocket;
	sockaddr_in dstAddr;
	WSADATA data;
	char _replyBuffer[BUFFER_SIZE];
	char _queryBuffer[BUFFER_SIZE];
};


#endif // !_TCP_IP_CLIENT_

