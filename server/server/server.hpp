#ifndef _TCPIP_SERVER_HPP
#define _TCPIP_SERVER_HPP
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <functional>

class server {
public:
	server(const std::function<void (const char*, char*)> &process, const int port);
	~server();
	void start();
private:
	enum{ BUFFER_SIZE = 1024 };
	sockaddr_in srcAddr;
	sockaddr_in dstAddr;
	WSAData data;
	int srcSocket;
	int dstSocket;
	int dstAddrSize;
	std::function<void (const char*, char*)> _process;
	char queryBuffer[BUFFER_SIZE];
	char replyBuffer[BUFFER_SIZE];
	int status;

};
#endif