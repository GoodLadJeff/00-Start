#pragma once
#include <iostream>
#include <cmath>
#include <random>

static class Utils
{
public:
    static std::pair<double, double> getRandomCoordinate(double arenaRadius) {
        // Generate a random angle in radians
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 2.0 * 3.14159265358979323846);
        double angle = dis(gen);

        // Generate a random radius within the arena
        std::uniform_real_distribution<> disRadius(0.0, arenaRadius);
        double radius = disRadius(gen);

        // Convert polar coordinates to Cartesian coordinates
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        return std::make_pair(x, y);
    }
};

