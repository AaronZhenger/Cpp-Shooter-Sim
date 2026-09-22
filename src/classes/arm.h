#pragma once
#include "Mechanism.h"

class Projectile;

class Arm : public Mechanism {
public:
    Arm(double radius, double angularVelocity, double pointOfReleaseRadians, 
        double efficiency, double exitX, double exitY);

    double radius;
    double angularVelocity;
    double pointOfReleaseRadians;

    double getExitVelocity(const Projectile& projectile) const override;
    double getExitAngle(const Projectile& projectile) const override;
    double getExitBackspin(const Projectile& projectile) const override;
};