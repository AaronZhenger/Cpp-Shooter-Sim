#include "Mechanism.h"
#include <iostream>

Mechanism::Mechanism(double efficiency, double exitX, double exitY)
    : efficiency(efficiency), exitX(exitX), exitY(exitY) {}

double Mechanism::getExitVelocity(const Projectile& projectile) const {
    std::cout << "Mechanism not configured yet\n";
    return 0.0;
}

double Mechanism::getExitAngle(const Projectile& projectile) const {
    std::cout << "Mechanism not configured yet\n";
    return 0.0;
}

double Mechanism::getExitBackspin(const Projectile& projectile) const {
    std::cout << "Mechanism not configured yet\n";
    return 0.0;
}