#pragma once
#include <string>
struct GLFWwindow;

// UTF-8 path; empty on cancellation. Failures throw an exception.
std::string selectUsdFile(GLFWwindow *owner);
