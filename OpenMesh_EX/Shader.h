#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include"Common.h"

class Shader
{
public:
	// the program ID
	unsigned int ID;

	Shader();
	Shader(const char* vertexPath, const char* fragmentPath);
	void use();

	// set uniform value
	void setUniformBool(const std::string& name, const bool value);
	void setUniformInt(const std::string& name, const int value);
	void setUniformFloat(const std::string& name, const float value);
	void setUniform4fv(const std::string& name, const float value[]);
	void setUniform4fv(const std::string& name, const float v1, const float v2, const float v3, const float v4);
	void setUniform3fv(const std::string& name, const float value[]);
	void setUniform3fv(const std::string& name, const glm::vec3& value);
	void setUniform3fv(const std::string& name, const float v1, const float v2, const float v3);
	void setUniformMatrix4fv(const std::string& name, const glm::mat4& trans);
};

