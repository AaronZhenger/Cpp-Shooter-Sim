#include "SlingShot.h"
#include "Projectile.h"
#include <cmath>

SlingShot::SlingShot(double horizontalDistance, double slingElasticity, double slingMass, 
                     double releaseAngle, double efficiency, double exitX, double exitY)
    : Mechanism(efficiency, exitX, exitY),
      horizontalDistance(horizontalDistance),
      slingElasticity(slingElasticity),
      slingMass(slingMass),
      releaseAngle(releaseAngle) {}

double SlingShot::getExitVelocity(const Projectile& projectile) const {
    return horizontalDistance
        * std::sqrt((efficiency * slingElasticity) / (projectile.mass + slingMass / 3.0));
}

double SlingShot::getExitAngle(const Projectile& projectile) const {
    return releaseAngle;
}

double SlingShot::getExitBackspin(const Projectile& projectile) const {
    return 0.0;
}