#include "SingleRotor.h"
#include "Projectile.h"

SingleRotor::SingleRotor(double flywheelRadius, double flywheelAngularVelocity, double releaseAngle, 
                         double efficiency, double exitX, double exitY)
    : Mechanism(efficiency, exitX, exitY),
      flywheelRadius(flywheelRadius),
      flywheelAngularVelocity(flywheelAngularVelocity),
      releaseAngle(releaseAngle) {}

double SingleRotor::getExitVelocity(const Projectile& projectile) const {
    return efficiency * (flywheelRadius * flywheelAngularVelocity) / 2.0;
}

double SingleRotor::getExitAngle(const Projectile& projectile) const {
    return releaseAngle;
}

double SingleRotor::getExitBackspin(const Projectile& projectile) const {
    return efficiency * (flywheelRadius * flywheelAngularVelocity) / (2.0 * projectile.radius);
}