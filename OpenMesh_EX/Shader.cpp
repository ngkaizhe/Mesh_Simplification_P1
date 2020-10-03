#include "Shader.h"

Shader::Shader() {
	ID = 0;
}

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
	// 1. retrieve the vertex/fragment source code from filePath
	std::string vertexCode;
	std::string fragmentCode;
	std::ifstream vShaderFile;
	std::ifstream fShaderFile;
	// ensure ifstream objects can throw exceptions:
	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// open files
		vShaderFile.open(vertexPath);
		fShaderFile.open(fragmentPath);
		std::stringstream vShaderStream, fShaderStream;
		// read file's buffer contents into streams
		vShaderStream << vShaderFile.rdbuf();
		fShaderStream << fShaderFile.rdbuf();
		// close file handlers
		vShaderFile.close();
		fShaderFile.close();
		// convert stream into string
		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}
	const char* vShaderCode = vertexCode.c_str();
	const char* fShaderCode = fragmentCode.c_str();

	// 2. compile shader
	// vertex shader
	unsigned int vShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vShader, 1, &vShaderCode, NULL);
	glCompileShader(vShader);
	// shader compilation error check
	int success;
	char infoLog[512];
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(vShader, 512, NULL, infoLog);
		std::cout << "\n\nVertex Shader Compilation Error -> " << infoLog << std::endl << std::endl;
	}

	//fragment shader
	unsigned int fShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fShader, 1, &fShaderCode, NULL);
	glCompileShader(fShader);
	// shader compilation error check
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fShader, 512, NULL, infoLog);
		std::cout << "\n\nFragment Shader Compilation Error -> " << infoLog << std::endl << std::endl;
	}

	// link shader program
	ID = glCreateProgram();
	glAttachShader(ID, vShader);
	glAttachShader(ID, fShader);
	glLinkProgram(ID);
	// program link error check
	glGetProgramiv(ID, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(ID, 512, NULL, infoLog);
		std::cout << "\n\nShader Program Linking Error -> " << infoLog << std::endl << std::endl;
	}

	//delete shader
	glDeleteShader(vShader);
	glDeleteShader(fShader);
}

void Shader::use() {
	// use program
	glUseProgram(ID);
}


// set uniform value
void Shader::setUniformBool(const std::string& name, const bool value) {
	this->use();
	glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

void Shader::setUniformInt(const std::string& name, const int value) {
	this->use();
	int a = glGetUniformLocation(ID, name.c_str());
	glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setUniformFloat(const std::string& name, const float value) {
	this->use();
	glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setUniform4fv(const std::string& name, const float value[]) {
	this->use();
	glUniform4f(glGetUniformLocation(ID, name.c_str()), value[0], value[1], value[2], value[3]);
}

void Shader::setUniform4fv(const std::string& name, const float v1, const float v2, const float v3, const float v4) {
	this->use();
	glUniform4f(glGetUniformLocation(ID, name.c_str()), v1, v2, v3, v4);
}

void Shader::setUniform3fv(const std::string& name, const float value[]) {
	this->use();
	glUniform3f(glGetUniformLocation(ID, name.c_str()), value[0], value[1], value[2]);
}

void Shader::setUniform3fv(const std::string& name, const glm::vec3& value) {
	this->use();
	glUniform3f(glGetUniformLocation(ID, name.c_str()), value[0], value[1], value[2]);
}

void Shader::setUniform3fv(const std::string& name, const float v1, const float v2, const float v3) {
	this->use();
	glUniform3f(glGetUniformLocation(ID, name.c_str()), v1, v2, v3);
}

void Shader::setUniformMatrix4fv(const std::string& name, const glm::mat4& trans) {
	this->use();
	glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &trans[0][0]);
}

