#pragma once
#include <Arduino.h>
struct FakeLog { template <typename... A> void printf(const char*, A...) {} void println(const char* = "") {} };
extern FakeLog Log;
