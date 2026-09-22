#pragma once
#include "Mechanism.h"

class Projectile;

class DualRotor : public Mechanism {
public:
    DualRotor(double bottomFlywheelRadius, double topFlywheelRadius, 
              double bottomFlywheelAngularVelocity, double topFlywheelAngularVelocity, 
              double releaseAngle, double efficiency, double exitX, double exitY);

    double bottomFlywheelRadius;
    double topFlywheelRadius;
    double bottomFlywheelAngularVelocity;
    double topFlywheelAngularVelocity;
    double releaseAngle;

    double getExitVelocity(const Projectile& projectile) const override;
    double getExitAngle(const Projectile& projectile) const override;
    double getExitBackspin(const Projectile& projectile) const override;
};