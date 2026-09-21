#pragma once

class Projectile {
    public:
        Projectile(double mass, double rotationalInertia, double crossSectionalArea, double radius, double dragCoefficient, double magnusCoefficient);

        virtual ~Projectile() = default;

        double mass;
        double rotationalInertia;
        double centerOfMassOffsetX;
        double centerOfMassOffsetY;
        double centerOfMassOffsetZ;
        double crossSectionalArea;
        double magnusCoefficient;
        double radius;
        double dragCoefficient;
        const double SPHERE_DRAG_COEFFICIENT = 0.47;
        const double CUBE_DRAG_COEFFICIENT = 1.05;
        const double FLAT_DRAG_COEFFICIENT = 1.28;
        const double BULLET_DRAG_COEFFICIENT = 0.30;
        const double CAR_DRAG_COEFFICIENT = 0.3;
};