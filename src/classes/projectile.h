#pragma once

class Projectile {
    public:
        Projectile(double mass, double rotationalInertia, double crossSectionalArea, double radius, double dragCoefficient, double magnusCoefficient);

        virtual ~Projectile() = default;

        double mass;
        double rotationalInertia;
        double crossSectionalArea;
        double magnusCoefficient;
        double radius;
        double dragCoefficient;
        static constexpr double SPHERE_DRAG_COEFFICIENT = 0.47;
        static constexpr double CUBE_DRAG_COEFFICIENT = 1.05;
        static constexpr double FLAT_DRAG_COEFFICIENT = 1.28;
        static constexpr double BULLET_DRAG_COEFFICIENT = 0.30;
        static constexpr double CAR_DRAG_COEFFICIENT = 0.3;
};