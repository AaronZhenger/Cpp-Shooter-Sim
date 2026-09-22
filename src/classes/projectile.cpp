#include "Projectile.h"
#include <iostream>

Projectile::Projectile(double mass, double rotationalInertia, double crossSectionalArea, double radius, double dragCoefficient, double magnusCoefficient)
    : mass(mass), rotationalInertia(rotationalInertia), crossSectionalArea(crossSectionalArea), radius(radius), dragCoefficient(dragCoefficient), magnusCoefficient(magnusCoefficient) {}