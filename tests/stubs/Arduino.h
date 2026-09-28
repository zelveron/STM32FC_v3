#pragma once
#include <cstdint>
#include <cstddef>
#define F(x) x
#define HEX 16
class Print {
public:
    template<class... T> void print(T...) {}
    template<class... T> void println(T...) {}
};
uint32_t micros();
uint32_t millis();
