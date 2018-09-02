
#include "CppUTest/CommandLineTestRunner.h"

#include <iostream>


 int main(int ac, char** av)
{
	std::cout << "Tests Begin\n";

	return CommandLineTestRunner::RunAllTests(ac, av);
}


















