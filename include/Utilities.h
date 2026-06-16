#ifndef __H_UTILITIES__
#define __H_UTILITIES__


#include <glad/glad.h>
//#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // for translate/rotate/scale
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/type_ptr.hpp>         // for value_ptr
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Config.hpp> 
#include <box2d/box2d.h>
#include <box2d/base.h>
#include <algorithm>
#include <iostream>
#include <optional>
#include <stdio.h>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <atomic>
#include <math.h>
#include <vector>
#include <memory>
#include <random>
#include <chrono>
#include <cmath>
#include <queue>
#include <mutex>
#include <deque>
#include <tuple>
#include <set>

//#define __TEST_CASE__
#define __USE_BULLET__



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif







enum Action_Selection_mode
{
    Inferense_mode,
    Train_mode,
    Random_mode
};




struct Action_Info
{
    bool RE_Active;
    bool LE_Active;
    bool RE_Boost;
    bool LE_Boost;
};

struct State_Info
{
    float   Lander_Pos_X;
    float   Lander_Pos_Y;
    float   Lander_LV_X;
    float   Lander_LV_Y;
    float   Lander_Ang_val;
    float   Lander_Ang_sgn;
    float   Lander_Ang_vel;
    //float   wind_speed_Direction_X;
    //float   wind_speed_Direction_Y;
    //float   Platform_Pos_X;
    //float   Platform_Pos_Y;
    float   fule_left;
};

struct Observation_Info
{
    State_Info state;
    float Reward;
    bool Done;
};

struct VaporParticle
{
    btRigidBody* body;
    float lifetime;
    float opacity;
    float size;
};

struct Rect_Shape_Info
{
    std::string         name = "";
	bool				is_static = false;
	sf::RectangleShape	SF_Shape;
	btRigidBody*		Blt_Body = nullptr;
	btVector3			Blt_Offset = {0, 0, 0};
};

struct Circ_Shape_Info
{
    std::string         name = "";
	bool				is_static = false;
	sf::CircleShape		SF_Shape;
	btRigidBody*		Blt_Body = nullptr;
	btVector3			Blt_Offset = {0, 0, 0};
};

struct Cnvx_Shape_Info
{
    std::string         name = "";
	bool				is_static = false;
	sf::ConvexShape		SF_Shape;
	btRigidBody*		Blt_Body = nullptr;
	btVector3			Blt_Offset = {0, 0, 0};
};




// --------- a Row of Replay Buffer ---------
struct Transition {
    State_Info state;
    Action_Info action;
    float reward;
    State_Info next_state;
    bool done;
};








b2Vec2 combine(b2Vec2 f1, b2Vec2 f2);

GLuint compileShader(GLenum type, const char* source);

GLuint createShaderProgram(const char* vertexSrc, const char* fragSrc);

std::vector<float> triangle_gen(float x1, float y1, float x2, float y2, float x3, float y3);

std::vector<float> rectangle_gen(float OrgX, float OrgY, float HalfWidth, float HalfHeight);

std::vector<float> cycle_gen(float OrgX, float OrgY, float Radios, int IndicesCnt);

std::vector<unsigned int> Indices_gen(int IndicesCnt);

sf::Vector2f toSFML(const btVector3& pos);


















#endif // !__H_UTILITIES__
