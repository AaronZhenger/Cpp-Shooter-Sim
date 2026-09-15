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

class Projectile {
    public:
        Projectile(double mass, double rotationalInertia, double centerOfMassOffsetX, double centerOfMassOffsetY, double centerOfMassOffsetZ, double crossSectionalArea, double radius, double dragCoefficient) {
            this->mass = mass;
            this->rotationalInertia = rotationalInertia;
            this->centerOfMassOffsetX = centerOfMassOffsetX;
            this->centerOfMassOffsetY = centerOfMassOffsetY;
            this->centerOfMassOffsetZ = centerOfMassOffsetZ;
            this->crossSectionalArea = crossSectionalArea;
            this->radius = radius;
            this->dragCoefficient = dragCoefficient;
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
    DualRotor m_mechanism(0.2, 0.1, 40.0, 40.0, 45, 0.85, 0.0, 0.0);

    Projectile m_projectile(0.2, 1.0, 0.0, 0.0, 0.0, 0.5, 0.2, 0.1);
    m_projectile.rotationalInertia = 2.0;

    Fluid m_fluid(0.2);

    const int seconds = 10;
    const int hz = 50;
    // std::vector<Position> positions;
    // positions.reserve(seconds * hz);
    // for (int i = 0; i < seconds * hz; i++) {
    //     double t = i * (1.0 / hz);
    //     positions.push_back({getPosX(m_projectile, m_fluid, m_mechanism, t), getPosY(m_projectile, m_fluid, m_mechanism, t)});
    // }

    // positions.reserve(6);
    // positions.push_back({-1, 0.0});
    // positions.push_back({-0.5, 0.4});
    // positions.push_back({-0.1, 0.6});
    // positions.push_back({0.1, 0.6});
    // positions.push_back({0.5, 0.4});
    // positions.push_back({1, 0.0});

    std::vector<Position> positions = {
        {-0.8000, 0.0288}, {-0.7809, 0.0468}, {-0.7618, 0.0647}, {-0.7429, 0.0824},
        {-0.7241, 0.0999}, {-0.7054, 0.1173}, {-0.6869, 0.1345}, {-0.6685, 0.1515},
        {-0.6502, 0.1684}, {-0.6320, 0.1851}, {-0.6139, 0.2016}, {-0.5960, 0.2180},
        {-0.5782, 0.2342}, {-0.5605, 0.2501}, {-0.5430, 0.2659}, {-0.5255, 0.2816},
        {-0.5082, 0.2970}, {-0.4911, 0.3122}, {-0.4740, 0.3273}, {-0.4571, 0.3421},
        {-0.4404, 0.3567}, {-0.4237, 0.3712}, {-0.4072, 0.3854}, {-0.3909, 0.3994},
        {-0.3746, 0.4132}, {-0.3585, 0.4268}, {-0.3426, 0.4402}, {-0.3267, 0.4534},
        {-0.3110, 0.4663}, {-0.2955, 0.4791}, {-0.2800, 0.4916}, {-0.2647, 0.5039},
        {-0.2496, 0.5159}, {-0.2346, 0.5277}, {-0.2197, 0.5393}, {-0.2050, 0.5507},
        {-0.1904, 0.5618}, {-0.1759, 0.5727}, {-0.1616, 0.5834}, {-0.1474, 0.5938},
        {-0.1333, 0.6040}, {-0.1194, 0.6139}, {-0.1056, 0.6236}, {-0.0920, 0.6330},
        {-0.0785, 0.6422}, {-0.0652, 0.6512}, {-0.0519, 0.6599}, {-0.0389, 0.6683},
        {-0.0259, 0.6765}, {-0.0131, 0.6845}, {-0.0005, 0.6921}, { 0.0121, 0.6996},
        { 0.0245, 0.7067}, { 0.0367, 0.7136}, { 0.0488, 0.7203}, { 0.0608, 0.7267},
        { 0.0726, 0.7328}, { 0.0843, 0.7387}, { 0.0959, 0.7443}, { 0.1073, 0.7496},
        { 0.1186, 0.7547}, { 0.1297, 0.7595}, { 0.1408, 0.7640}, { 0.1516, 0.7683},
        { 0.1624, 0.7723}, { 0.1730, 0.7760}, { 0.1835, 0.7795}, { 0.1938, 0.7827},
        { 0.2040, 0.7856}, { 0.2141, 0.7883}, { 0.2241, 0.7907}, { 0.2339, 0.7928},
        { 0.2436, 0.7946}, { 0.2531, 0.7962}, { 0.2625, 0.7975}, { 0.2718, 0.7985},
        { 0.2810, 0.7993}, { 0.2900, 0.7998}, { 0.2990, 0.8000}, { 0.3077, 0.7999},
        { 0.3164, 0.7996}, { 0.3249, 0.7990}, { 0.3334, 0.7981}, { 0.3417, 0.7970},
        { 0.3498, 0.7956}, { 0.3579, 0.7939}, { 0.3658, 0.7919}, { 0.3736, 0.7897},
        { 0.3813, 0.7872}, { 0.3889, 0.7845}, { 0.3963, 0.7814}, { 0.4037, 0.7781},
        { 0.4109, 0.7746}, { 0.4180, 0.7707}, { 0.4250, 0.7666}, { 0.4319, 0.7623},
        { 0.4386, 0.7577}, { 0.4453, 0.7528}, { 0.4519, 0.7476}, { 0.4583, 0.7422},
        { 0.4646, 0.7365}, { 0.4709, 0.7306}, { 0.4770, 0.7244}, { 0.4830, 0.7180},
        { 0.4890, 0.7113}, { 0.4948, 0.7043}, { 0.5005, 0.6971}, { 0.5061, 0.6896},
        { 0.5117, 0.6819}, { 0.5171, 0.6739}, { 0.5225, 0.6657}, { 0.5277, 0.6573},
        { 0.5329, 0.6486}, { 0.5379, 0.6396}, { 0.5429, 0.6304}, { 0.5478, 0.6210},
        { 0.5526, 0.6113}, { 0.5573, 0.6014}, { 0.5619, 0.5912}, { 0.5665, 0.5809},
        { 0.5709, 0.5703}, { 0.5753, 0.5594}, { 0.5796, 0.5483}, { 0.5839, 0.5370},
        { 0.5880, 0.5255}, { 0.5921, 0.5138}, { 0.5961, 0.5018}, { 0.6001, 0.4896},
        { 0.6039, 0.4772}, { 0.6077, 0.4646}, { 0.6115, 0.4517}, { 0.6151, 0.4387},
        { 0.6187, 0.4254}, { 0.6223, 0.4120}, { 0.6258, 0.3983}, { 0.6292, 0.3845},
        { 0.6326, 0.3704}, { 0.6359, 0.3561}, { 0.6391, 0.3417}, { 0.6423, 0.3270},
        { 0.6455, 0.3122}, { 0.6486, 0.2972}, { 0.6517, 0.2819}, { 0.6547, 0.2665},
        { 0.6576, 0.2510}, { 0.6605, 0.2352}, { 0.6634, 0.2193}, { 0.6663, 0.2032},
        { 0.6691, 0.1869}, { 0.6718, 0.1705}, { 0.6746, 0.1539}, { 0.6773, 0.1371},
        { 0.6799, 0.1202}, { 0.6826, 0.1031}, { 0.6852, 0.0859}, { 0.6878, 0.0685},
        { 0.6903, 0.0510}, { 0.6928, 0.0333}, { 0.6953, 0.0155}, { 0.6978, -0.0025},
        { 0.7003, -0.0206}, { 0.7028, -0.0388}, { 0.7052, -0.0572}, { 0.7076, -0.0757},
        { 0.7100, -0.0943}, { 0.7125, -0.1130}, { 0.7148, -0.1319}, { 0.7172, -0.1508},
        { 0.7196, -0.1699}, { 0.7220, -0.1891}, { 0.7244, -0.2084}, { 0.7268, -0.2278},
        { 0.7291, -0.2473}, { 0.7315, -0.2669}, { 0.7339, -0.2866}, { 0.7363, -0.3063},
        { 0.7387, -0.3262}, { 0.7411, -0.3462}, { 0.7436, -0.3662}, { 0.7460, -0.3863},
        { 0.7484, -0.4065}, { 0.7509, -0.4268}, { 0.7534, -0.4471}, { 0.7559, -0.4675},
        { 0.7584, -0.4879}, { 0.7610, -0.5084}, { 0.7636, -0.5290}, { 0.7662, -0.5496},
        { 0.7688, -0.5703}, { 0.7715, -0.5910}, { 0.7741, -0.6118}, { 0.7769, -0.6326},
        { 0.7796, -0.6534}, { 0.7824, -0.6743}, { 0.7852, -0.6952}, { 0.7881, -0.7161},
        { 0.7910, -0.7371}, { 0.7940, -0.7580}, { 0.7970, -0.7790}, { 0.8000, -0.8000}
    };

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

    GLuint shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLuint VAO, VBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Position), positions.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.07, 0.13, 0.17, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINE_STRIP, 0, positions.size());
        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}


//MOI, COM, MASS, MAGNUS, DRAG, COP, Area