#include "bounce.h"

Bounce::Bounce(double surfaceAngleWithHorizontal, double surfaceVelocity, double angleOfIncidence, double efficiency, double exitX, double exitY)
    : Mechanism(efficiency, exitX, exitY), surfaceAngleWithHorizontal(surfaceAngleWithHorizontal), angleOfIncidence(angleOfIncidence), surfaceVelocity(surfaceVelocity) {}

double Bounce::getExitVelocity(const Projectile& projectile) const {
    return efficiency
        * surfaceVelocity;
}

double Bounce::getExitAngle(const Projectile& projectile) const {
    return 2 * surfaceAngleWithHorizontal - angleOfIncidence;
}