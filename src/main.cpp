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
#include "projectile.h"
#include "mechanism.h"
#include "fluid.h"
#include "dualrotor.h"

const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
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
        - ((std::pow(k, 2) - std::pow(omega, 2)) * 9.8085) / (std::pow(k, 2) + std::pow(omega, 2))
    ) / (std::pow(k, 2) + std::pow(omega, 2));

    double curvature_1 = a_x * (std::exp(-k * time) * cos(omega * time) - 1);
    double curvature_2 = a_y * (std::exp(-k * time) * sin(omega * time));
    
    return mechanism.exitX + drift + curvature_1 - curvature_2;
}

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
}

struct Position {
    double x, y;
};

struct Character {
    GLuint textureID;
    int sizeX, sizeY;
    int bearingX, bearingY;
    unsigned int advance;
};

int main() {
    DualRotor m_mechanism(0.2, 0.1, 40.0, 40.0, 3.14/4, 0.85, 0.0, 0.0);
    Projectile m_projectile(0.2, 1.0, 0.5, 0.2, 0.1, 0.03);
    Fluid m_fluid(0.2);

    const int seconds = 10;
    const int hz = 50;
    
    std::vector<Position> rawPositions;
    rawPositions.reserve(seconds * hz);
    double startY = getPosY(m_projectile, m_fluid, m_mechanism, 0.0);

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
        float normY = static_cast<float>(-0.25 + 1.25 * ((p.y - minY) / (maxY - minY)));
        positions.push_back({normX, normY});
    }

    std::vector<Position> gridPositions;
    std::vector<double> xLabels;
    std::vector<double> yLabels;
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

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 800, "Projectile Simulator", NULL, NULL);
    if (window == NULL) {
        std::cout << "Window failed to Launch\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glViewport(0, 0, 800, 800);

    FT_Library ft;
    if (FT_Init_FreeType(&ft)) {
        std::cerr << "no lib" << std::endl;
        return -1;
    }

    FT_Face face;
    if (FT_New_Face(ft, "src/fonts/ArialBold.ttf", 0, &face)) {
        std::cerr << "no font" << std::endl;  
        return -1;
    }

    FT_Set_Pixel_Sizes(face, 0, 48);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    std::map<char, Character> characters;
    for (unsigned char c = 0; c < 128; c++) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            std::cerr << "no glyph" << c << std::endl;
            continue;
        }

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
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Position), positions.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_DOUBLE, GL_FALSE, sizeof(Position), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &gridVAO); glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, gridPositions.size() * sizeof(Position), gridPositions.data(), GL_STATIC_DRAW);
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

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(gridShaderProgram);
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridPositions.size()));
        
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(positions.size()));

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

        glfwSwapBuffers(window);
        glfwPollEvents();
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

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}