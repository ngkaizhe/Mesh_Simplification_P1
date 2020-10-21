#include "MyTimer.h"


void MyTimer::Start(std::string message) {
	if (message != "") std::cout << message << '\n';
	std::cout << "Start Counting Timer!\n";
	startTime = std::chrono::steady_clock::now();
}

void MyTimer::Flag(std::string message, int lineNumberBeforeMessage, int lineNumberAfterMessage) {
	for (int i = 0; i < lineNumberBeforeMessage; i++) std::cout << '\n';

	if(message != "")	std::cout << message << '\n';

	std::cout << "Current Pass Time => ";
	std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - startTime;
	int hours = 0, minutes = 0;
	double seconds = std::fmod(elapsed_seconds.count(), 60);
	hours = elapsed_seconds.count() / 60 / 60;
	minutes = int(elapsed_seconds.count() / 60) % 60;
	std::cout << "Hour: " << hours << ", Minutes: " << minutes << ", Seconds: " << seconds << "\n\n";

	for (int i = 0; i < lineNumberAfterMessage; i++) std::cout << '\n';
}