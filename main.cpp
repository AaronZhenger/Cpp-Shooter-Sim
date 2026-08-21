#include <iostream>

class Mechanism {
    public:
        double efficiency;
        double bottomFlywheelRadius;
        double topFlywheelRadius;
        double bottomFlywheelAngularVelocity;
        double topFlywheelAngularVelocity;

        double getExitVelocity() {
            return efficiency
                * (topFlywheelRadius * topFlywheelAngularVelocity + bottomFlywheelRadius * bottomFlywheelAngularVelocity) / 2;
        }

        

    double time = 2;
};

class Projectile {
    public:
        double rotationalInertia;
};

double getExitBackspin(Mechanism mechanism, Projectile projectile) {
            return mechanism.efficiency
                * (mechanism.bottomFlywheelRadius * mechanism.bottomFlywheelAngularVelocity
                    - mechanism.topFlywheelRadius * mechanism.topFlywheelAngularVelocity)
                / projectile.rotationalInertia / 2
                * mechanism.time;
};

int main() {
    Mechanism m_mechanism;
    m_mechanism.efficiency = 0.85;
    m_mechanism.bottomFlywheelRadius = 0.2;
    m_mechanism.topFlywheelRadius = 0.1;
    m_mechanism.bottomFlywheelAngularVelocity = 40.0;
    m_mechanism.topFlywheelAngularVelocity = 40.0;

    Projectile m_projectile;
    m_projectile.rotationalInertia = 2.0;

    std::cout << m_mechanism.getExitVelocity() << '\n';
    std::cout << getExitBackspin(m_mechanism, m_projectile) << '\n';
    return 0;
}