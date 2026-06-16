
#include "Utilities.h"
#include "Constants.h"

b2Vec2 combine(b2Vec2 f1, b2Vec2 f2)
{
	b2Vec2 tmp;
	tmp.x = sqrtf(powf(f1.x, 2) + powf(f2.x, 2));
	tmp.y = sqrtf(powf(f1.y, 2) + powf(f2.y, 2));
	return tmp;
}


GLuint compileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader Compilation Error: " << infoLog << std::endl;
    }
    return shader;
}


GLuint createShaderProgram(const char* vertexSrc, const char* fragSrc)
{
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSrc);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    int success;
    char infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Program Linking Error: " << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}


std::vector<float> triangle_gen(float x1, float y1, float x2, float y2, float x3, float y3)
{
    std::vector<float> tmp;
    tmp.push_back(x1);
    tmp.push_back(y1);
    tmp.push_back(x2);
    tmp.push_back(y2);
    tmp.push_back(x3);
    tmp.push_back(y3);
    return tmp;
}


std::vector<float> rectangle_gen(float OrgX, float OrgY, float HalfWidth, float HalfHeight)
{
    std::vector<float> tmp;
    tmp.push_back(OrgX - HalfWidth);
    tmp.push_back(OrgY - HalfHeight);
    tmp.push_back(OrgX - HalfWidth);
    tmp.push_back(OrgY + HalfHeight);
    tmp.push_back(OrgX + HalfWidth);
    tmp.push_back(OrgY + HalfHeight);
    tmp.push_back(OrgX + HalfWidth);
    tmp.push_back(OrgY - HalfHeight);
    return tmp;
}


std::vector<float> cycle_gen(float OrgX, float OrgY, float Radios, int IndicesCnt)
{
    std::vector<float> tmp;
    for (float rad = 0; rad < 360; rad = rad + 360 / IndicesCnt)
    {
        tmp.push_back(OrgX + Radios * cos(glm::radians(rad)));
        tmp.push_back(OrgY + Radios * sin(glm::radians(rad)));
    }
    return tmp;
}


std::vector<unsigned int> Indices_gen(int IndicesCnt)
{
    std::vector<unsigned int> tmp;
    tmp.push_back(0);
    tmp.push_back(1);
    tmp.push_back(2);
    for (int i = 3; i <= IndicesCnt; i++)
    {
        tmp.push_back(i-1);
        tmp.push_back(i);
        tmp.push_back(0);
    }
    return tmp;
}

sf::Vector2f toSFML(const btVector3& pos)
{
    return { pos.getX() * SCALE, Window_Height - pos.getY() * SCALE };
}

