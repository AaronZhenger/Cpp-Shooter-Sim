#pragma once
#include "Mechanism.h"

class Projectile;

class SingleRotor : public Mechanism {
public:
    SingleRotor(double flywheelRadius, double flywheelAngularVelocity, double releaseAngle, 
                double efficiency, double exitX, double exitY);

    double flywheelRadius;
    double flywheelAngularVelocity;
    double releaseAngle;

    double getExitVelocity(const Projectile& projectile) const override;
    double getExitAngle(const Projectile& projectile) const override;
    double getExitBackspin(const Projectile& projectile) const override;
};