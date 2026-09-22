#include "DualRotor.h"
#include "Projectile.h"

DualRotor::DualRotor(double bottomFlywheelRadius, double topFlywheelRadius, 
                     double bottomFlywheelAngularVelocity, double topFlywheelAngularVelocity, 
                     double releaseAngle, double efficiency, double exitX, double exitY)
    : Mechanism(efficiency, exitX, exitY),
      bottomFlywheelRadius(bottomFlywheelRadius),
      topFlywheelRadius(topFlywheelRadius),
      bottomFlywheelAngularVelocity(bottomFlywheelAngularVelocity),
      topFlywheelAngularVelocity(topFlywheelAngularVelocity),
      releaseAngle(releaseAngle) {}

double DualRotor::getExitVelocity(const Projectile& projectile) const {
    return efficiency * (topFlywheelRadius * topFlywheelAngularVelocity + bottomFlywheelRadius * bottomFlywheelAngularVelocity) / 2.0;
}

double DualRotor::getExitAngle(const Projectile& projectile) const {
    return releaseAngle;
}

double DualRotor::getExitBackspin(const Projectile& projectile) const {
    return efficiency * (bottomFlywheelRadius * bottomFlywheelAngularVelocity - topFlywheelRadius * topFlywheelAngularVelocity) / projectile.radius / 2.0;
}