#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <vector>
#include <cmath>
#include <numbers>
#include <map>
#include <sstream>
#include <iomanip>
#include <memory>
#include <algorithm>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "projectile.h"
#include "mechanism.h"
#include "fluid.h"
#include "dualrotor.h"
#include "singlerotor.h"
#include "arm.h"
#include "slingshot.h"
#include "bounce.h"

const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
"   gl_PointSize = 8.0;\n"
"}\0";

const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor; \n"
"void main()\n"
"{\n"
"   FragColor = vec4(0.9f, 1.0f, 1.0f, 1.0f);\n"
"}\n\0";

const char* gridFragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor; \n"
"void main()\n"
"{\n"
"   FragColor = vec4(0.5f, 0.5f, 0.5f, 1.0f);\n"
"}\n\0";

const char* textVertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec4 aVertex;\n"
"out vec2 TexCoords;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(aVertex.xy, 0.0, 1.0);\n"
"   TexCoords = aVertex.zw;\n"
"}\0";

const char* textFragmentShaderSource = "#version 330 core\n"
"in vec2 TexCoords;\n"
"out vec4 FragColor;\n"
"uniform sampler2D textTexture;\n"
"uniform vec3 textColor;\n"
"void main()\n"
"{\n"
"   vec4 sampled = vec4(1.0, 1.0, 1.0, texture(textTexture, TexCoords).r);\n"
"   FragColor = vec4(textColor, 1.0) * sampled;\n"
"}\n\0";

double getDragCoefficient(const Projectile& projectile, const Fluid& fluid) {
    return 0.5 * fluid.density * projectile.dragCoefficient * projectile.crossSectionalArea;
}

double getMagnusConstant(const Projectile& projectile, const Mechanism& mechanism) {
    return (projectile.magnusCoefficient * mechanism.getExitBackspin(projectile)) / projectile.mass;
}

constexpr double g = 9.8085;

double getPosX(const Projectile& projectile, const Fluid& fluid, const Mechanism& mechanism, double time) {
    double omega = getMagnusConstant(projectile, mechanism);
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;

    if (k == 0.0 && omega == 0.0)
        return mechanism.exitX + mechanism.getExitVelocity(projectile) * std::cos(mechanism.getExitAngle(projectile)) * time;

    double denom = k * k + omega * omega;
    double drift = omega * g * time / denom;

    double v0 = mechanism.getExitVelocity(projectile);
    double angle = mechanism.getExitAngle(projectile);

    double a_x = (omega * v0 * std::sin(angle) - k * v0 * std::cos(angle) + (2 * k * omega * g) / denom) / denom;
    double a_y = (-omega * v0 * std::cos(angle) - k * v0 * std::sin(angle) - ((k * k - omega * omega) * g) / denom) / denom;

    double curvature_1 = a_x * (std::exp(-k * time) * std::cos(omega * time) - 1.0);
    double curvature_2 = a_y * (std::exp(-k * time) * std::sin(omega * time));

    return mechanism.exitX + drift + curvature_1 - curvature_2;
}

double getPosY(const Projectile& projectile, const Fluid& fluid, const Mechanism& mechanism, double time) {
    double omega = getMagnusConstant(projectile, mechanism);
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;
    
    if (k == 0.0 && omega == 0.0) {
        double v0 = mechanism.getExitVelocity(projectile);
        double angle = mechanism.getExitAngle(projectile);
        return mechanism.exitY + v0 * std::sin(angle) * time - 0.5 * g * time * time;
    }
    
    double denom = std::pow(k, 2) + std::pow(omega, 2);
    double drift = k * g * time / denom;
    
    double v0 = mechanism.getExitVelocity(projectile);
    double angle = mechanism.getExitAngle(projectile);

    double a_x = (omega * v0 * std::sin(angle) - k * v0 * std::cos(angle) + (2 * k * omega * g) / denom) / denom;
    double a_y = (-omega * v0 * std::cos(angle) - k * v0 * std::sin(angle) - ((std::pow(k, 2) - std::pow(omega, 2)) * g) / denom) / denom;

    double curvature_1 = a_y * (std::exp(-k * time) * std::cos(omega * time) - 1.0);
    double curvature_2 = a_x * (std::exp(-k * time) * std::sin(omega * time));
    
    return mechanism.exitY - drift + curvature_1 + curvature_2;
}

double getPosX(const Projectile& projectile, const Fluid& fluid, double v0, double angle, double omega, double exitX, double time) {
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;

    if (k == 0.0 && omega == 0.0)
        return exitX + v0 * std::cos(angle) * time;

    double denom = k * k + omega * omega;
    double drift = omega * g * time / denom;

    double a_x = (omega * v0 * std::sin(angle) - k * v0 * std::cos(angle) + (2 * k * omega * g) / denom) / denom;
    double a_y = (-omega * v0 * std::cos(angle) - k * v0 * std::sin(angle) - ((k * k - omega * omega) * g) / denom) / denom;

    double curvature_1 = a_x * (std::exp(-k * time) * std::cos(omega * time) - 1.0);
    double curvature_2 = a_y * (std::exp(-k * time) * std::sin(omega * time));

    return exitX + drift + curvature_1 - curvature_2;
}

double getPosY(const Projectile& projectile, const Fluid& fluid, double v0, double angle, double omega, double exitY, double time) {
    double k = getDragCoefficient(projectile, fluid) / projectile.mass;
    
    if (k == 0.0 && omega == 0.0) {
        return exitY + v0 * std::sin(angle) * time - 0.5 * g * time * time;
    }
    
    double denom = std::pow(k, 2) + std::pow(omega, 2);
    double drift = k * g * time / denom;

    double a_x = (omega * v0 * std::sin(angle) - k * v0 * std::cos(angle) + (2 * k * omega * g) / denom) / denom;
    double a_y = (-omega * v0 * std::cos(angle) - k * v0 * std::sin(angle) - ((std::pow(k, 2) - std::pow(omega, 2)) * g) / denom) / denom;

    double curvature_1 = a_y * (std::exp(-k * time) * std::cos(omega * time) - 1.0);
    double curvature_2 = a_x * (std::exp(-k * time) * std::sin(omega * time));
    
    return exitY - drift + curvature_1 + curvature_2;
}

double findTime(const Projectile& projectile, const Fluid& fluid, double v0, double angle, double omega, double exitX, double targetX) {
    double t = 0.5;
    for (int iter = 0; iter < 12; ++iter) {
        double x = getPosX(projectile, fluid, v0, angle, omega, exitX, t);
        double diff = x - targetX;
        if (std::abs(diff) < 1e-4) break;

        double dt = 1e-5;
        double dxdt = (getPosX(projectile, fluid, v0, angle, omega, exitX, t + dt) - x) / dt;
        if (std::abs(dxdt) < 1e-6) break;

        t -= diff / dxdt;
        if (t <= 0.0) t = 0.001;
    }
    return t;
}

struct SimpleConfig {
    public:
        SimpleConfig(double theta, double v) : theta(theta), v(v) {};
    double theta, v;
};

struct DetailedConfig {
    public:
        DetailedConfig(SimpleConfig config, double t, double d) : config(config), t(t), d(d) {};
    SimpleConfig config;
    double t, d;
};

struct Position {
    public:
        Position(double x, double y) : x(x), y(y) {};
    double x, y;
};

struct Character {
    GLuint textureID;
    int sizeX, sizeY;
    int bearingX, bearingY;
    unsigned int advance;
};

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 800, "Projectile Simulator", NULL, NULL);
    if (window == NULL) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glViewport(0, 0, 800, 800);

    FT_Library ft;
    if (FT_Init_FreeType(&ft))
        return -1;

    FT_Face face;
    if (FT_New_Face(ft, "src/fonts/ArialBold.ttf", 0, &face))
        return -1;

    FT_Set_Pixel_Sizes(face, 0, 48);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    std::map<char, Character> characters;
    for (unsigned char c = 0; c < 128; c++) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER))
            continue;

        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RED,
            face->glyph->bitmap.width, face->glyph->bitmap.rows,
            0, GL_RED, GL_UNSIGNED_BYTE, face->glyph->bitmap.buffer
        );

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        Character character = {
            texture,
            static_cast<int>(face->glyph->bitmap.width),
            static_cast<int>(face->glyph->bitmap.rows),
            static_cast<int>(face->glyph->bitmap_left),
            static_cast<int>(face->glyph->bitmap_top),
            static_cast<unsigned int>(face->glyph->advance.x)
        };
        characters.insert(std::pair<char, Character>(c, character));
    }
    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true); 
    ImGui_ImplOpenGL3_Init("#version 330");

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    GLuint gridFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(gridFragmentShader, 1, &gridFragmentShaderSource, NULL);
    glCompileShader(gridFragmentShader);

    GLuint textVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(textVertexShader, 1, &textVertexShaderSource, NULL);
    glCompileShader(textVertexShader);

    GLuint textFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(textFragmentShader, 1, &textFragmentShaderSource, NULL);
    glCompileShader(textFragmentShader);

    GLuint shaderProgram = glCreateProgram();
    GLuint gridShaderProgram = glCreateProgram();
    GLuint textShaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glAttachShader(gridShaderProgram, vertexShader);
    glAttachShader(gridShaderProgram, gridFragmentShader);
    glLinkProgram(gridShaderProgram);

    glAttachShader(textShaderProgram, textVertexShader);
    glAttachShader(textShaderProgram, textFragmentShader);
    glLinkProgram(textShaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(gridFragmentShader);
    glDeleteShader(textVertexShader);
    glDeleteShader(textFragmentShader);

    GLuint VAO, VBO, gridVAO, gridVBO, textVAO, textVBO;

    glGenVertexArrays(1, &VAO); glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 0, NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &gridVAO); glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &textVAO); glGenBuffers(1, &textVBO);
    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    auto renderText = [&](const std::string& text, float x, float y, float scale, float color[3]) {
        glUseProgram(textShaderProgram);
        glUniform3f(glGetUniformLocation(textShaderProgram, "textColor"), color[0], color[1], color[2]);
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(textVAO);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        for (char c : text) {
            Character ch = characters[c];

            float xpos = x + ch.bearingX * scale;
            float ypos = y - (ch.sizeY - ch.bearingY) * scale;

            float w = ch.sizeX * scale;
            float h = ch.sizeY * scale;

            float x0 = (xpos / static_cast<float>(800)) * 2.0f - 1.0f;
            float y0 = (ypos / static_cast<float>(800)) * 2.0f - 1.0f;
            float x1 = ((xpos + w) / static_cast<float>(800)) * 2.0f - 1.0f;
            float y1 = ((ypos + h) / static_cast<float>(800)) * 2.0f - 1.0f;

            float vertices[6][4] = {
                { x0, y1, 0.0f, 0.0f },
                { x0, y0, 0.0f, 1.0f },
                { x1, y0, 1.0f, 1.0f },

                { x0, y1, 0.0f, 0.0f },
                { x1, y0, 1.0f, 1.0f },
                { x1, y1, 1.0f, 0.0f }
            };

            glBindTexture(GL_TEXTURE_2D, ch.textureID);
            glBindBuffer(GL_ARRAY_BUFFER, textVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            glDrawArrays(GL_TRIANGLES, 0, 6);

            x += (ch.advance >> 6) * scale;
        }
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_BLEND);
    };

    float labelColor[3] = { 0.8f, 0.8f, 0.8f };
    const int seconds = 10;
    const int hz = 200;

    std::vector<Position> rawPositions;
    std::vector<Position> positions;
    std::vector<Position> gridPositions;
    std::vector<double> xLabels;
    std::vector<double> yLabels;

    std::vector<SimpleConfig> simpleConfigs;
    std::vector<DetailedConfig> advConfigs;

    static int currentMechanism = 0;
    const char* mechanisms = "Dual Rotor\0Single Rotor\0Arm\0Slingshot\0Bounce\0";

    static int currentMode = 0;
    const char* modes = "Horizontal\0Raw Time\0Prediction\0";
    
    static int fluidPreset = 1;
    const char* fluids = "None\0Air\0Water\0";
    double fluidDensity = 0.0;

    static int predictionMode = 0;
    const char * predictionModes = "Time\0Distance\0Clearance\0";

    double minAngle = 0;
    double maxAngle = 60.0;
    double minVel = 0;
    double maxVel = 40.0;

    double f_x = 5.0;
    double f_y = 0.0;
    double o_x = 4.0;
    double o_y = 2.0;

    double time = 1.0;

    double efficiency = 0.85;
    double initialX = 0.0;
    double initialY = 0.0;

    double p_mass = 0.8;
    double p_rotInertia = 1.0;
    double p_area = 0.5;
    double p_radius = 0.2;
    double p_dragC = 0.47;
    double p_magnusC = 0.03;

    double d_bfr = 0.2;
    double d_tfr = 0.1;
    double d_bfav = 40.0;
    double d_tfav = 40.0;
    double d_ra = 45;

    double s_fr = 0.2;
    double s_fav = 40.0;
    double s_ra = 3.14/4;

    double a_r = 0.5;
    double a_av = 6.0;
    double a_ra = 45;

    double ss_x = 0.5;
    double ss_e = 40.0;
    double ss_m = .2;
    double ss_ra = 45;

    double b_theta = 0.0;
    double b_v = 10.0;
    double b_phi = -45;

    double predictionVelocity;
    double predictionAngle;

    double lastFrameTime = glfwGetTime();
    float animTime = 0.0f;
    bool isPlaying = true;
    float playbackSpeed = 1.0f;
    bool loopAnimation = true;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        double currentFrameTime = glfwGetTime();
        double deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        if (isPlaying) {
            animTime += static_cast<float>(deltaTime) * playbackSpeed;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        std::unique_ptr<Mechanism> m_mechanism;

        Projectile m_projectile(p_mass, p_rotInertia, p_area, p_radius, p_dragC, p_magnusC * .1);
        Fluid m_fluid(fluidDensity);

        int prevMech = currentMechanism;
        int prevMode = currentMode;

        ImGui::Begin("Controls");
        ImGui::SetWindowPos(ImVec2(50.0, 20.0), ImGuiCond_Once);
        ImGui::SetWindowSize(ImVec2(240.0, 150.0), ImGuiCond_Always);
        if (currentMode == 1)
            ImGui::SetWindowSize(ImVec2(240.0, 170.0), ImGuiCond_Always);
        ImGui::Text("FPS: %.0f", ImGui::GetIO().Framerate);
        ImGui::Text("Mode:        ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0);
        ImGui::Combo("##mode", &currentMode, modes);
        ImGui::Text("Mechanism:   ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0);
        ImGui::Combo("##mech", &currentMechanism, mechanisms);
        if (currentMode == 1) {
            ImGui::Text("Flight Time: ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##time", &time);
        }
        ImGui::Separator();
        if (ImGui::Button(isPlaying ? "Pause" : "Play")) isPlaying = !isPlaying;
        ImGui::SameLine();
        if (ImGui::Button("Reset")) animTime = 0.0f;
        ImGui::SameLine();
        ImGui::Checkbox("Loop", &loopAnimation);
        ImGui::SetNextItemWidth(120.0);
        ImGui::SliderFloat("Speed", &playbackSpeed, 0.1f, 5.0f);

        if (prevMech != currentMechanism || prevMode != currentMode) {
            animTime = 0.0f;
        }
        ImGui::End();

        if (currentMode == 2) {
            ImGui::Begin("Predictions");
            ImGui::SetWindowPos(ImVec2(540.0, 20.0), ImGuiCond_Once);
            ImGui::SetWindowSize(ImVec2(240.0, 200.0), ImGuiCond_Always);
            if (predictionMode == 2) {
                ImGui::SetWindowSize(ImVec2(240.0, 245.0), ImGuiCond_Always);
            }
            ImGui::Separator();
            ImGui::Combo("##pm", &predictionMode, predictionModes);
            ImGui::Text("Min Angle:   ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##mina", &minAngle);
            ImGui::Text("Max Angle:   ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##maxa", &maxAngle);
            ImGui::Text("Min Velocity:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##minv", &minVel);
            ImGui::Text("Max Velocity:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##maxv", &maxVel);
            ImGui::Text("Final X:     ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##fx", &f_x);
            ImGui::Text("Final Y:     ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0);
            ImGui::InputDouble("##fy", &f_y);
            if (predictionMode == 2) {
                ImGui::Text("Obstacle X:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0);
                ImGui::InputDouble("##ox", &o_x);
                ImGui::Text("Obstacle Y:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0);
                ImGui::InputDouble("##oy", &o_y);
            }
            ImGui::End();
        }

        ImGui::Begin("Fluid");
        ImGui::SetWindowPos(ImVec2(20.0, 540.0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(240.0, 60.0), ImGuiCond_Always);
        if (fluidPreset == 0)
            ImGui::SetWindowSize(ImVec2(240.0, 80.0), ImGuiCond_Always);
        ImGui::Text("Density Preset:    ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::Combo("##preset", &fluidPreset, fluids);
        if (fluidPreset == 0) {
            ImGui::Text("Density:           ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80.0);
            ImGui::InputDouble("##density", &fluidDensity);
        } 
        else if (fluidPreset == 1)
            fluidDensity = Fluid::air;
        else fluidDensity = Fluid::water;
        ImGui::End();

        ImGui::Begin("Mechanism");
        ImGui::SetWindowPos(ImVec2(20.0, 610.0), ImGuiCond_Always);
        if (fluidPreset == 0)
            ImGui::SetWindowPos(ImVec2(20.0, 630.0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(240.0, 107.0), ImGuiCond_Always);
        ImGui::Text("Efficiency:        ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##eff", &efficiency);
        ImGui::Separator();
        ImGui::Text("Initial X:         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##x", &initialX);
        ImGui::Text("Initial Y:         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##y", &initialY);
        ImGui::End();

        ImGui::Begin("Projectile");
        ImGui::SetWindowPos(ImVec2(540.0, 540.0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(240.0, 193.0), ImGuiCond_Always);
        ImGui::Text("Mass:              ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##m", &p_mass);
        ImGui::Text("Rotational Inertia:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##RI", &p_rotInertia);
        ImGui::Text("Cross Sectional");
        ImGui::Text("Area:              ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##CSA", &p_area);
        ImGui::Text("Radius:            ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##r", &p_radius);
        ImGui::Separator();
        ImGui::Text("Drag Coefficient:  ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##dc", &p_dragC);
        ImGui::Text("Magnus Coefficient:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0);
        ImGui::InputDouble("##mc", &p_magnusC);
        ImGui::End();

        switch (currentMechanism) {
            case 0 :
                ImGui::Begin("Dual Rotor");
                ImGui::SetWindowPos(ImVec2(280.0, 540.0), ImGuiCond_Always);
                ImGui::SetWindowSize(ImVec2(240.0, 220.0), ImGuiCond_Always);
                ImGui::Text("Bottom Flywheel    ");
                ImGui::Text("Radius:            ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##bfr", &d_bfr);
                ImGui::Text("Top Flywheel       ");
                ImGui::Text("Radius:            ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##tfr", &d_tfr);
                ImGui::Text("Bottom Flywheel    ");
                ImGui::Text("Angular Velocity:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##bfav", &d_bfav);
                ImGui::Text("Top Flywheel       ");
                ImGui::Text("Angular Velocity:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##tfav", &d_tfav);
                ImGui::Separator();
                ImGui::Text("Release Angle:     ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##ra", &d_ra);
                ImGui::End();
                m_mechanism = std::make_unique<DualRotor>(d_bfr, d_tfr, d_bfav, d_tfav, d_ra * 3.1415 / 180.0, efficiency, initialX, initialY);
                break;
            case 1 :
                ImGui::Begin("Single Rotor");
                ImGui::SetWindowPos(ImVec2(280.0, 540.0), ImGuiCond_Always);
                ImGui::SetWindowSize(ImVec2(240.0, 123.0), ImGuiCond_Always);
                ImGui::Text("Flywheel Radius:   ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##fr", &s_fr);
                ImGui::Text("Flywheel Angular   ");
                ImGui::Text("Velocity:          ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##fav", &s_fav);
                ImGui::Separator();
                ImGui::Text("Release Angle:     ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##ra", &s_ra);
                ImGui::End();
                m_mechanism = std::make_unique<SingleRotor>(s_fr, s_fav, s_ra * 3.1415 / 180.0, efficiency, initialX, initialY);
                break;
            case 2 :
                ImGui::Begin("Arm");
                ImGui::SetWindowPos(ImVec2(280.0, 540.0), ImGuiCond_Always);
                ImGui::SetWindowSize(ImVec2(240.0, 109.0), ImGuiCond_Always);
                ImGui::Text("Radius:            ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##r", &a_r);
                ImGui::Text("Angular Velocity:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##av", &a_av);
                ImGui::Separator();
                ImGui::Text("Release Angle:     ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##ra", &a_ra);
                ImGui::End();
                m_mechanism = std::make_unique<Arm>(a_r, a_av, a_ra * 3.1415 / 180.0, efficiency, initialX, initialY);
                break;
            case 3 :
                ImGui::Begin("Slingshot");
                ImGui::SetWindowPos(ImVec2(280.0, 540.0), ImGuiCond_Always);
                ImGui::SetWindowSize(ImVec2(240.0, 130.0), ImGuiCond_Always);
                ImGui::Text("Distance Pulled:   ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##x", &ss_x);
                ImGui::Text("Elasticity:        ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##e", &ss_e);
                ImGui::Text("Band Mass:         ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##m", &ss_m);
                ImGui::Separator();
                ImGui::Text("Release Angle:     ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##ra", &ss_ra);
                ImGui::End();
                m_mechanism = std::make_unique<SlingShot>(ss_x, ss_e, ss_m, ss_ra * 3.1415 / 180.0, efficiency, initialX, initialY);
                break;
            case 4 :
                ImGui::Begin("Bounce");
                ImGui::SetWindowPos(ImVec2(280.0, 540.0), ImGuiCond_Always);
                ImGui::SetWindowSize(ImVec2(240.0, 125.0), ImGuiCond_Always);
                ImGui::Text("Surface Angle      ");
                ImGui::Text("Offset:            ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##t", &b_theta);
                ImGui::Text("Contact Velocity:  ");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##v", &b_v);
                ImGui::Separator();
                ImGui::Text("Angle Of Incidence:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0);
                ImGui::InputDouble("##p", &b_phi);

                ImGui::End();
                m_mechanism = std::make_unique<Bounce>(b_theta * 3.1415 / 180.0, b_v, b_phi * 3.1415 / 180.0, efficiency, initialX, initialY);
                break;
        }

        rawPositions.clear();
        positions.clear();
        gridPositions.clear();
        xLabels.clear();
        yLabels.clear();

        simpleConfigs.clear();
        advConfigs.clear();

        for (int a = minAngle; a <= maxAngle; a+=0.1) {
            for (int v = minVel; v <= maxVel; v+=0.02) {
                for (int t = 0; t < 2; t+=0.005) {
                    if (predictionMode == 2) {

                    } else {
                        if (std::abs(getPosX(m_projectile, m_fluid, predictionVelocity, predictionAngle, getMagnusConstant(m_projectile, *m_mechanism), m_mechanism.get()->exitX, t)-f_x) < 0.01
                            && std::abs(getPosY(m_projectile, m_fluid, predictionVelocity, predictionAngle, getMagnusConstant(m_projectile, *m_mechanism), m_mechanism.get()->exitY, t)-f_y) < 0.01) {
                    
                        }
                    }
                }
            }
        }

        rawPositions.reserve(seconds * hz);
        double startY = getPosY(m_projectile, m_fluid, *m_mechanism, 0.0);

        double iterations = time * hz;
        if (currentMode == 0)
            iterations = seconds * hz;

        for (int i = 0; i < iterations; i++) {
            double t = i * (1.0 / hz);
            Position current = {
                getPosX(m_projectile, m_fluid, *m_mechanism, t),
                getPosY(m_projectile, m_fluid, *m_mechanism, t)
            };

            if (currentMode == 2) {
                current = {
                    getPosX(m_projectile, m_fluid, predictionVelocity, predictionAngle, getMagnusConstant(m_projectile, *m_mechanism), m_mechanism.get()->exitX, t),
                    getPosY(m_projectile, m_fluid, predictionVelocity, predictionAngle, getMagnusConstant(m_projectile, *m_mechanism), m_mechanism.get()->exitY, t)
                };
            }

            rawPositions.push_back(current);

            if (currentMode == 0 && i > 1 && current.y <= startY)
                break;
        }

        double totalFlightDuration = rawPositions.size() / static_cast<double>(hz);
        if (animTime > totalFlightDuration) {
            if (loopAnimation) {
                animTime = 0.0f;
            } else {
                animTime = static_cast<float>(totalFlightDuration);
                isPlaying = false;
            }
        }

        if (!rawPositions.empty()) {
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
            
            positions.reserve(rawPositions.size());
            for (const auto& p : rawPositions) {
                float normX = static_cast<float>(-1.0 + 2.0 * ((p.x - minX) / (maxX - minX)));
                float normY = static_cast<float>(-0.25 + 1.25 * ((p.y - minY) / (maxY - minY)));
                positions.push_back({normX, normY});
            }

            double gridSpacing = maxDimension / 10.0;
            
            for (double x = std::floor(minX / gridSpacing) * gridSpacing; x <= maxX; x += gridSpacing) {
                float normX = static_cast<float>(-1.0 + 2.0 * ((x - minX) / (maxX - minX)));
                gridPositions.push_back({normX, -0.25});
                gridPositions.push_back({normX, 1.0});
                xLabels.push_back(x);
            }

            for (double y = std::floor(minY / gridSpacing) * gridSpacing + gridSpacing; y <= maxY; y += gridSpacing) {
                float normY = static_cast<float>(-0.25 + 1.25 * ((y - minY) / (maxY - minY)));
                gridPositions.push_back({-1.0, normY});
                gridPositions.push_back({1.0, normY});
                yLabels.push_back(y);
            }

            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Position), positions.data(), GL_DYNAMIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
            glBufferData(GL_ARRAY_BUFFER, gridPositions.size() * sizeof(Position), gridPositions.data(), GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        if (!gridPositions.empty()) {
            glUseProgram(gridShaderProgram);
            glBindVertexArray(gridVAO);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridPositions.size()));
        }
        
        if (!positions.empty()) {
            size_t visibleCount = std::min(positions.size(), static_cast<size_t>(animTime * hz));

            glUseProgram(shaderProgram);
            glBindVertexArray(VAO);

            if (visibleCount > 1) {
                glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(visibleCount));
            }

            if (visibleCount > 0) {
                glEnable(GL_PROGRAM_POINT_SIZE);
                glDrawArrays(GL_POINTS, static_cast<GLsizei>(visibleCount - 1), 1);
            }
        }

        if (!rawPositions.empty()) {
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
            minX = centerX - padding; maxX = centerX + padding;
            minY = centerY - padding; maxY = centerY + padding;

            for (double val : xLabels) {
                float normX = static_cast<float>(-1.0 + 2.0 * ((val - minX) / (maxX - minX)));
                float pixelX = (normX + 1.0f) / 2.0f * 800;

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(1) << val;
                renderText(ss.str(), pixelX - 12.0f, 280.0f, 0.4f, labelColor);
            }

            for (double val : yLabels) {
                float normY = static_cast<float>(-0.25 + 1.25 * ((val - minY) / (maxY - minY)));
                float pixelY = (normY + 1.0f) / 2.0f * 800;

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(1) << val;
                renderText(ss.str(), 10.0f, pixelY - 5.0f, 0.4f, labelColor);
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteVertexArrays(1, &textVAO);
    glDeleteBuffers(1, &textVBO);
    glDeleteProgram(shaderProgram);
    glDeleteProgram(gridShaderProgram);
    glDeleteProgram(textShaderProgram);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}