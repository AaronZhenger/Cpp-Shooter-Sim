#pragma once
#include "mechanism.h"

class Mechanism;
class Projectile;

class Bounce : public Mechanism {
    public:
        Bounce(double surfaceAngleWithHorizontal, double surfaceVelocity, double angleOfIncidence, double efficiency, double exitX, double exitY);

        virtual ~Bounce() = default;

        double surfaceAngleWithHorizontal;
        double surfaceVelocity;
        double angleOfIncidence;

        virtual double getExitVelocity(const Projectile& projectile) const override;
        virtual double getExitAngle(const Projectile& projectile) const override;
};