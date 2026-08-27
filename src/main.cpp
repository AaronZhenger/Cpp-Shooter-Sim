#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

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
"   FragColor= vec4(0.8f, 0.3f, 0.02f, 1.0f);\n"
"}\n\0";

// class Mechanism {
//     public:
//         double efficiency;
//         double bottomFlywheelRadius;
//         double topFlywheelRadius;
//         double bottomFlywheelAngularVelocity;
//         double topFlywheelAngularVelocity;

//         double getExitVelocity() {
//             return efficiency
//                 * (topFlywheelRadius * topFlywheelAngularVelocity + bottomFlywheelRadius * bottomFlywheelAngularVelocity) / 2;
//         }

        

//     double time = 2;
// };

// class Projectile {
//     public:
//         double rotationalInertia;
// };

// double getExitBackspin(Mechanism mechanism, Projectile projectile) {
//             return mechanism.efficiency
//                 * (mechanism.bottomFlywheelRadius * mechanism.bottomFlywheelAngularVelocity
//                     - mechanism.topFlywheelRadius * mechanism.topFlywheelAngularVelocity)
//                 / projectile.rotationalInertia / 2
//                 * mechanism.time;
// };

int main() {
    // Mechanism m_mechanism;
    // m_mechanism.efficiency = 0.85;
    // m_mechanism.bottomFlywheelRadius = 0.2;
    // m_mechanism.topFlywheelRadius = 0.1;
    // m_mechanism.bottomFlywheelAngularVelocity = 40.0;
    // m_mechanism.topFlywheelAngularVelocity = 40.0;

    // Projectile m_projectile;
    // m_projectile.rotationalInertia = 2.0;

    // std::cout << m_mechanism.getExitVelocity() << '\n';
    // std::cout << getExitBackspin(m_mechanism, m_projectile) << '\n';

    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLfloat vertices[] = {
        -0.5, -0.5, 0.0,
        0.5, -0.5, 0.0,
        0.0, 0.5, 0.0
    };

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
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3* sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.07, 0.13, 0.17, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
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