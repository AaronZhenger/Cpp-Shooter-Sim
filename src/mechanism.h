#pragma once

class Projectile;

class Mechanism {
    protected:
        Mechanism(double efficiency, double exitX, double exitY);
    
    public:
        virtual ~Mechanism() = default;
    
        double efficiency;
        double exitX;
        double exitY;
    
        virtual double getExitVelocity(const Projectile& projectile) const;
        virtual double getExitAngle(const Projectile& projectile) const;
        virtual double getExitBackspin(const Projectile& projectile) const;
};