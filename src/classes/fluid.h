#pragma once

class Fluid {
public:
    explicit Fluid(double density);

    double density;
    static constexpr double air = 1.225;
    static constexpr double water = 1000;
};