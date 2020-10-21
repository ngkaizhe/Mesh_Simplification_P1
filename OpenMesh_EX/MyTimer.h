#pragma once
#include<chrono>
#include<string>
#include<iostream>
#include<cmath>

static class MyTimer
{
public:
	void Start(std::string message="");
	void Flag(std::string message="", int lineNumberBeforeMessage=0, int lineNumberAfterMessage=0);

private:
	std::chrono::steady_clock::time_point startTime;
};



