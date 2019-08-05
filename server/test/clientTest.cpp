#include "client.hpp"
#include "position.hpp"
#include <memory>
#include <iostream>

int main() {
	auto c = std::unique_ptr<client>(new client("172.16.99.160", 2001));
	position pos;
	c->queryPosition(&pos);
	std::cout << "(1): " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	Sleep(1000);
	c->queryPosition(&pos);
	std::cout << "(2): " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	Sleep(1000);
	c->queryPosition(&pos);
	std::cout << "(3): " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
	getchar();

}