#pragma once

#include "Mechanism.h"

class Projectile;

class SlingShot : public Mechanism {
public:
    SlingShot(double horizontalDistance, double slingElasticity, double slingMass, 
              double releaseAngle, double efficiency, double exitX, double exitY);

    double horizontalDistance;
    double slingElasticity;
    double slingMass;
    double releaseAngle;

    double getExitVelocity(const Projectile& projectile) const override;
    double getExitAngle(const Projectile& projectile) const override;
    double getExitBackspin(const Projectile& projectile) const override;
};