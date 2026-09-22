#include "Arm.h"
#include "Projectile.h"

Arm::Arm(double radius, double angularVelocity, double pointOfReleaseRadians, 
         double efficiency, double exitX, double exitY)
    : Mechanism(efficiency, exitX, exitY),
      radius(radius),
      angularVelocity(angularVelocity),
      pointOfReleaseRadians(pointOfReleaseRadians) {}

double Arm::getExitVelocity(const Projectile& projectile) const {
    return efficiency * radius * angularVelocity;
}

double Arm::getExitAngle(const Projectile& projectile) const {
    return pointOfReleaseRadians + 3.14159265358979323846 / 2.0;
}

double Arm::getExitBackspin(const Projectile& projectile) const {
    return angularVelocity;
}