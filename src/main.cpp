#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <numbers>

const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"void main()\n"
"{\n"
"   gl_Position= vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
"}\0";
const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor; \n"
"void main()\n"
"{\n"
"   FragColor= vec4(0.9f, 1.0f, 1.0f, 1.0f);\n"
"}\n\0";

const char* gridFragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor; \n"
"void main()\n"
"{\n"
"   FragColor= vec4(0.5f, 0.5f, 0.5f, 1.0f);\n"
"}\n\0";

class Projectile {
    public:
        Projectile(double mass, double rotationalInertia, double crossSectionalArea, double radius, double dragCoefficient, double magnusCoefficient) {
            this->mass = mass;
            this->rotationalInertia = rotationalInertia;
            this->crossSectionalArea = crossSectionalArea;
            this->radius = radius;
            this->dragCoefficient = dragCoefficient;
            this->magnusCoefficient = magnusCoefficient;
        }

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

class Mechanism {
    protected:
        Mechanism(double efficiency, double exitX, double exitY) {
            this->efficiency = efficiency;
            this->exitX = exitX;
            this->exitY = exitY;
        }
    public:
        double efficiency;
        double exitX;
        double exitY;
        virtual double getExitVelocity(const Projectile& projectile) const {
            std::cout << "Mechanism not configured yet";
            return 0;
        }
        virtual double getExitAngle(const Projectile& projectile) const {
            std::cout << "Mechanism not configured yet";
            return 0;
        }
        virtual double getExitBackspin(const Projectile& projectile) const {
            std::cout << "Mechanism not configured yet";
            return 0;
        }
};

class Bounce : public Mechanism {
    public:
        Bounce(double surfaceAngleWithHorizontal, double surfaceVelocity, double slingangleOfIncidenceMass, double efficiency, double exitX, double exitY) : Mechanism(efficiency, exitX, exitY) {
            this->surfaceAngleWithHorizontal = surfaceAngleWithHorizontal;
            this->surfaceVelocity = surfaceVelocity;
            this->angleOfIncidence = angleOfIncidence;
        }

        double surfaceAngleWithHorizontal;
        double surfaceVelocity;
        double angleOfIncidence;

        double getExitVelocity(const Projectile& projectile) const override{
            return efficiency
                * surfaceVelocity;
        }

        double getExitAngle(const Projectile& projectile) const override{
            return 2 * surfaceAngleWithHorizontal - angleOfIncidence;
        }
};

class SlingShot : public Mechanism {
    public:
        SlingShot(double horizontalDistance, double slingElasticity, double slingMass, double releaseAngle, double efficiency, double exitX, double exitY) : Mechanism(efficiency, exitX, exitY) {
            this->horizontalDistance = horizontalDistance;
            this->slingElasticity = slingElasticity;
            this->slingMass = slingMass;
            this->releaseAngle = releaseAngle;
        }

        double horizontalDistance;
        double slingElasticity;
        double slingMass;
        double releaseAngle;

        double getExitVelocity(const Projectile& projectile) const override {
            return horizontalDistance
                * std::sqrt(
                    (efficiency * slingElasticity) 
                    / (projectile.mass + slingMass/3));
        }

        double getExitAngle(const Projectile& projectile) const override {
            return releaseAngle;
        }

        double getExitBackspin(const Projectile& projectile) const override {
            return 0.0;
        }
};

class Arm : public Mechanism {
    public:
        Arm(double radius, double angularVelocity, double pointOfReleaseRadians, double efficiency, double exitX, double exitY) : Mechanism(efficiency, exitX, exitY) {
            this->radius = radius;
            this->angularVelocity = angularVelocity;
            this->pointOfReleaseRadians = pointOfReleaseRadians;
        }

        double radius;
        double angularVelocity;
        double pointOfReleaseRadians;

        double getExitVelocity(const Projectile& projectile) const override {
            return efficiency * radius * angularVelocity;
        }

        double getExitAngle(const Projectile& projectile) const override {
            return pointOfReleaseRadians + 3.141592653589793238462643383279502884 / 2.0;
        }

        double getExitBackspin(const Projectile& projectile) const override {
            return angularVelocity;
        }
};

class SingleRotor : public Mechanism {
    public:
        SingleRotor(double flywheelRadius, double flywheelAngularVelocity, double releaseAngle, double efficiency, double exitX, double exitY) : Mechanism(efficiency, exitX, exitY) {
            this->flywheelRadius = flywheelRadius;
            this->flywheelAngularVelocity = flywheelAngularVelocity;
            this->releaseAngle = releaseAngle;
        }

        double flywheelRadius;
        double flywheelAngularVelocity;
        double releaseAngle;

        double getExitVelocity(const Projectile& projectile) const override {
            return efficiency
                * (flywheelRadius * flywheelAngularVelocity) / 2;
        }

        double getExitAngle(const Projectile& projectile) const override {
            return releaseAngle;
        }
        
        double getExitBackspin(const Projectile& projectile) const override {
            return efficiency
                * (flywheelRadius * flywheelAngularVelocity) / (2 * projectile.radius);
        }
};

class DualRotor : public Mechanism {
    public:
        DualRotor(double bottomFlywheelRadius, double topFlywheelRadius, double bottomFlywheelAngularVelocity, double topFlywheelAngularVelocity, double releaseAngle, double efficiency, double exitX, double exitY) : Mechanism(efficiency, exitX, exitY) {
            this->bottomFlywheelRadius = bottomFlywheelRadius;
            this->topFlywheelRadius = topFlywheelRadius;
            this->bottomFlywheelAngularVelocity = bottomFlywheelAngularVelocity;
            this->topFlywheelAngularVelocity = topFlywheelAngularVelocity;
            this->releaseAngle = releaseAngle;
        }

        double bottomFlywheelRadius;
        double topFlywheelRadius;
        double bottomFlywheelAngularVelocity;
        double topFlywheelAngularVelocity;
        double releaseAngle;

        double getExitVelocity(const Projectile& projectile) const override {
            return efficiency
                * (topFlywheelRadius * topFlywheelAngularVelocity + bottomFlywheelRadius * bottomFlywheelAngularVelocity) / 2;
        }

        double getExitAngle(const Projectile& projectile) const override {
            return releaseAngle;
        }

        double getExitBackspin(const Projectile& projectile) const override {
            return efficiency
                * (bottomFlywheelRadius * bottomFlywheelAngularVelocity
                    - topFlywheelRadius * topFlywheelAngularVelocity)
                / projectile.radius / 2;
            };
};

class Fluid {
    public:
        Fluid(double density) {
            this->density = density;
        }

        double density;
};

double getDragCoefficient(const Projectile& projectile, const Fluid& fluid) {
    return 0.5 * fluid.density * projectile.dragCoefficient * projectile.crossSectionalArea;
};

double getMagnusConstant(const Projectile& projectile, const Mechanism& mechanism) {
    return (projectile.magnusCoefficient * mechanism.getExitBackspin(projectile)) / projectile.mass;
}

double getPosX(const Projectile& projectile, const Fluid& fluid, const Mechanism& mechanism, double time) {
    double omega = getMagnusConstant(projectile, mechanism);
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;

    double drift = omega * 9.8085 * time / (std::pow(k, 2) + std::pow(omega, 2));

    double a_x = (omega * mechanism.getExitVelocity(projectile) * sin(mechanism.getExitAngle(projectile))
        - k * mechanism.getExitVelocity(projectile) * cos(mechanism.getExitAngle(projectile))
        + (2 * k * omega * 9.8085) / (std::pow(k, 2) + std::pow(omega, 2))
    ) / (std::pow(k, 2) + std::pow(omega, 2));

    double a_y = (-omega * mechanism.getExitVelocity(projectile) * cos(mechanism.getExitAngle(projectile))
        - k * mechanism.getExitVelocity(projectile) * sin(mechanism.getExitAngle(projectile))
        + (2 * k * omega * 9.8085) / (std::pow(k, 2) + std::pow(omega, 2))
    ) / (std::pow(k, 2) + std::pow(omega, 2));

    double curvature_1 = a_x * (std::exp(-k * time) * cos(omega * time) - 1);

    double curvature_2 = a_y * (std::exp(-k * time) * sin(omega * time));
    
    return mechanism.exitX + drift + curvature_1 - curvature_2;
};

double getPosY(const Projectile& projectile, const Fluid& fluid, const Mechanism& mechanism, double time) {
    double omega = getMagnusConstant(projectile, mechanism);
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;
    
    double drift = k * 9.8085 * time / (std::pow(k, 2) + std::pow(omega, 2));
    
    double a_x = (omega * mechanism.getExitVelocity(projectile) * sin(mechanism.getExitAngle(projectile))
        - k * mechanism.getExitVelocity(projectile) * cos(mechanism.getExitAngle(projectile))
        + (2 * k * omega * 9.8085) / (std::pow(k, 2) + std::pow(omega, 2))
    ) / (std::pow(k, 2) + std::pow(omega, 2));

    double a_y = (-omega * mechanism.getExitVelocity(projectile) * cos(mechanism.getExitAngle(projectile))
        - k * mechanism.getExitVelocity(projectile) * sin(mechanism.getExitAngle(projectile))
        - ((std::pow(k, 2) - std::pow(omega, 2)) * 9.8085) / (std::pow(k, 2) + std::pow(omega, 2))
    ) / (std::pow(k, 2) + std::pow(omega, 2));

    double curvature_1 = a_y * (std::exp(-k * time) * cos(omega * time) - 1);

    double curvature_2 = a_x * (std::exp(-k * time) * sin(omega * time));
    
    return mechanism.exitY - drift + curvature_1 + curvature_2;
};

struct Position {
    double x, y;
};

int main() {
    DualRotor m_mechanism(0.2, 0.1, 40.0, 40.0, 3.14/4, 0.85, 0.0, 0.0);

    Projectile m_projectile(0.2, 1.0, 0.5, 0.2, 0.1, 0.03);

    Fluid m_fluid(0.2);

    const int seconds = 10; //saslkj max
    const int hz = 50;
    
    std::vector<Position> rawPositions;
    rawPositions.reserve(seconds * hz);
    double startY = getPosY(m_projectile, m_fluid, m_mechanism, 0.0); // = m_mechanism.exitY;

    for (int i = 0; i < seconds * hz; i++) {
        double t = i * (1.0 / hz);
        Position current = {getPosX(m_projectile, m_fluid, m_mechanism, t), getPosY(m_projectile, m_fluid, m_mechanism, t)};

        rawPositions.push_back(current);
        if (i > 1 && current.y <= startY)
            break;
    }

    double minX = rawPositions[0].x;
    double maxX = rawPositions[0].x;
    double minY = rawPositions[0].y;
    double maxY = rawPositions[0].y;
    for (const auto& p : rawPositions) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }

    double maxDimension = std::max({maxX - minX, maxY - minY, 0.00001});
    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    double padding = maxDimension * 1.1 / 2.0;

    minX = centerX - padding;
    maxX = centerX + padding;
    minY = centerY - padding;
    maxY = centerY + padding;

    
    std::vector<Position> positions;
    positions.reserve(rawPositions.size());
    for (const auto& p : rawPositions) {
        float normX = static_cast<float>(-1.0 + 2.0 * ((p.x - minX) / (maxX - minX)));
        float normY = static_cast<float>(-1.0 + 2.0 * ((p.y - minY) / (maxY - minY)));
        positions.push_back({normX, normY});
    }

    std::vector<Position> gridPositions;
    double gridSpacing = maxDimension / 10.0;
    
    for (double x = std::floor(minX/gridSpacing) * gridSpacing; x <= maxX; x += gridSpacing) {
        float normX = static_cast<float>(-1.0 + 2.0 * ((x - minX) / (maxX - minX)));
        gridPositions.push_back({normX, -1.0});
        gridPositions.push_back({normX, 1.0});
    }

    for (double y = std::floor(minY / gridSpacing) * gridSpacing; y <= maxY; y += gridSpacing) {
        float normY = static_cast<float>(-1.0 + 2.0 * ((y - minY) / (maxY - minY)));
        gridPositions.push_back({-1.0, normY});
        gridPositions.push_back({1.0, normY});
    }

    std::cout << m_mechanism.getExitVelocity(m_projectile) << '\n';
    std::cout << m_mechanism.getExitBackspin(m_projectile) << '\n';

    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 800, "Projectile Simulator", NULL, NULL);
    if (window == NULL) {
        std::cout << "Window failed to Launch" << '\n';
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    gladLoadGL();

    glClearColor(0.07, 0.13, 0.17, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window);

    glViewport(0, 0, 800, 800);

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    GLuint gridFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(gridFragmentShader, 1, &gridFragmentShaderSource, NULL);
    glCompileShader(gridFragmentShader);

    GLuint shaderProgram = glCreateProgram();
    GLuint gridShaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glAttachShader(gridShaderProgram, vertexShader);
    glAttachShader(gridShaderProgram, gridFragmentShader);
    glLinkProgram(gridShaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(gridFragmentShader);

    GLuint VAO, VBO;
    GLuint gridVAO, gridVBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Position), positions.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glBindVertexArray(gridVAO);

    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);

    glBufferData(GL_ARRAY_BUFFER, gridPositions.size() * sizeof(Position), gridPositions.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);

    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.07, 0.13, 0.17, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(gridShaderProgram);
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, gridPositions.size());
        
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINE_STRIP, 0, positions.size());
        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteProgram(shaderProgram);
    glDeleteProgram(gridShaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}


//MOI, COM, MASS, MAGNUS, DRAG, COP, Area