#pragma once 

#include <random>

inline std::mt19937& rng() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;           // returns an lvalue reference — OK for dist(rng())
}