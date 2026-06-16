#pragma once

#include "Utilities.h"
#include "Constants.h"
#include "Vapor.h"



class LLEnv
{
public:
	LLEnv(std::mt19937& gen, int Max_FPS, float timestep, float BFC, float NFC, float EFC, float MF);
	~LLEnv();


	// --- Environment Functions ---
	void Add_to_fule(float incoming_fule);
	void Take_Action(Action_Info info);
	void Step();
	void Get_Observation(Observation_Info& info);
	void Print_Observation_info(Observation_Info Oinfo, Action_Info Ainfo);
	void restart();
	void Set_Engine_Angle(float REng, float LEng);
	bool Active_window_isOpen();
	void Close_Active_Window();
	void Disable_FPS_Limit();
	void Set_FPS_limit(int FPS);
	void Hide_Window();
	void Show_Window();

	// --- Visualization ---
	void Create_Window(std::string str);
	void Clear_Window();
	void Polling_Window_Event();
	void draw_all();
	void Render();


	//	Movie Maker Functions
	void Create_Video(std::string strx);
	void Add_a_Frame();
	void Release_video();

private:
	void Apply_Action();
	void update_fule();
	void ClearUp();
	void Initialize();
	bool is_Done(Observation_Info info);
	bool is_Out_of_Scope(Observation_Info info);
	bool is_Make_Contact_with_Platform(Observation_Info info);
	bool is_Make_Contact_with_Mountain(Observation_Info info);
	float Calculate_Distance();
	float Claculate_Reward(Observation_Info	New_info);
	float clampf(float v, float lo, float hi);

	// internal
	std::mt19937& gen;
	std::uniform_int_distribution<>			intDist;
	std::uniform_real_distribution<>		floatDist;
	std::normal_distribution<>				normDist;
	std::string								str;

	float									TimeStep;
	float									Lunar_Fule;
	float									Base_fule_consumption;
	float									normal_fule_consumption;
	float									extra_fule_consumption;
	float									maximum_fule;
	float									Last_dist;
	float									New_dist;
	float									Next_Goal;
	Observation_Info						Old_info;

	float									Right_Eng_Direction;
	float									Left_Eng_Direction;
	int										LEngIdx;
	int										REngIdx;
	int										LunrIdx;
	int										VaprDec;
	int										RVpCntr;
	int										LVpCntr;

	bool									Force_Applied_to_Right;
	bool									Force_Applied_to_Left;
	bool									Extra_Force_Applied_to_Right;
	bool									Extra_Force_Applied_to_Left;
	bool									last_RE_Active;
	bool									last_LE_Active;
	bool									last_RE_Boost;
	bool									last_LE_Boost;
	bool									Advanced;


	std::vector<Rect_Shape_Info>			All_Rect_Shapes_Info;
	std::vector<Circ_Shape_Info>			All_Circ_Shapes_Info;
	std::vector<Cnvx_Shape_Info>			All_Cnvx_Shapes_Info;

	EngineVapor* REV;
	EngineVapor* LEV;


	// ----------------------------
	// Setup Video Writer
	// ----------------------------
	std::optional <cv::VideoWriter> writer;



	// --- SFML Variables ---
	//sf::RenderWindow* window;
	std::optional<sf::RenderWindow> window;
	int	MFPS;

	// --- Bullet Variables
	btDefaultCollisionConfiguration* collisionConfig;
	btCollisionDispatcher* dispatcher;
	btDbvtBroadphase* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* world;
	std::vector<btCollisionShape*>			Shapes;
	//	Mountain
	btTriangleMesh* mountainMesh;
	btBvhTriangleMeshShape* mountainShape;
	btRigidBody* mountainBody;
	btDefaultMotionState* mountainMotionState;
	//	Lunar Lander
	btCompoundShape* LL_compound;
	btBoxShape* LL_body;
	btBoxShape* LL_Arm;
	btBoxShape* LL_leg;
	btBoxShape* LL_Eng;
	btDefaultMotionState* motionState;
	btRigidBody* LunLander;



	// --- Visualization ---
	void draw_vapor();



	// --- SFML Functions ---
	void Make_a_Rectangle(float Hw, float Hh, Rect_Shape_Info& info, sf::Color color);
	void Make_a_Circle(float Radius, Circ_Shape_Info& info, sf::Color color);
	void Make_a_Cnvx(std::vector<sf::Vector2f> nodes, Cnvx_Shape_Info& info, sf::Color color);

	// --- Bullet Functions ---
	void Let_be_Universe();
	void Let_be_Gravity(btVector3 Gravity);
	void Let_be_a_Ground();
	void Let_be_a_Rectangle(float Hw, float Hh, float Hd, float Px, float Py, float Pz, float mass, sf::Color color);
	void Let_be_a_Circle(float Radius, float Px, float Py, float Pz, float mass, sf::Color color);
	void Let_be_a_Mountain();
	void Let_be_a_LunarLander(float Px, float Py, float Pz, float mass, sf::Color color);
	void Let_be_a_Platform_Flags();
	void Let_be_a_Engine_Vapor();


	sf::Vector2f True_Translation(const btVector3& localOffset, btTransform trans);
	void ApplyEngineThrust(const btVector3& localEnginePos, float localNozzleAngleDeg, float Force_Multiplier);
};



