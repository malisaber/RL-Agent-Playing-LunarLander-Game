#include "Lunar_Lander_Environment.h"

LLEnv::LLEnv(std::mt19937& gen, int Max_FPS, float timestep, float BFC, float NFC, float EFC, float MF) :
    gen(gen),
    intDist(1, 10),
    floatDist(0.0, 1.0),
    normDist(0.0, 1.0),
    collisionConfig(nullptr),
    dispatcher(nullptr),
    broadphase(nullptr),
    solver(nullptr),
    world(nullptr),
    mountainMesh(nullptr),
    mountainShape(nullptr),
    mountainBody(nullptr),
    mountainMotionState(nullptr),
    LL_compound(nullptr),
    LL_body(nullptr),
    LL_Arm(nullptr),
    LL_leg(nullptr),
    LL_Eng(nullptr),
    motionState(nullptr),
    REV(nullptr),
    LEV(nullptr),
    LunLander(nullptr),
    MFPS(Max_FPS),
    Advanced(false)
{
    Base_fule_consumption = BFC;
    normal_fule_consumption = NFC;
    extra_fule_consumption = EFC;
    maximum_fule = MF;

    Right_Eng_Direction = -30.0f;
    Left_Eng_Direction = +30.0f;

    TimeStep = timestep;
    Lunar_Fule = 0;

    LEngIdx = -1;
    REngIdx = -1;
    LunrIdx = -1;
    VaprDec = Vapor_Decay;
    RVpCntr = 0;
    LVpCntr = 0;
    str = "";

    Force_Applied_to_Right = false;
    Force_Applied_to_Left = false;
    Extra_Force_Applied_to_Right = false;
    Extra_Force_Applied_to_Left = false;

    last_RE_Active = false;
    last_LE_Active = false;
    last_RE_Boost = false;
    last_LE_Boost = false;

    Last_dist = 0;
    New_dist = 0;
    Next_Goal = Window_Height / SCALE - Platform_Y;

    Old_info.Done = false;
    Old_info.Reward = 0;
    Old_info.state.Lander_Pos_X = 0;
    Old_info.state.Lander_Pos_Y = 0;
    Old_info.state.Lander_LV_X = 0;
    Old_info.state.Lander_LV_Y = 0;
    Old_info.state.Lander_Ang_val = 0;
    Old_info.state.Lander_Ang_sgn = 0;
    Old_info.state.Lander_Ang_vel = 0;
    //Old_info.state.wind_speed_Direction_X = 0;
    //Old_info.state.wind_speed_Direction_Y = 0;
    //Old_info.state.Platform_Pos_X = 0;
    //Old_info.state.Platform_Pos_Y = 0;
    Old_info.state.fule_left = 0;
}

LLEnv::~LLEnv()
{
    ClearUp();
    delete world;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
}

void LLEnv::Add_to_fule(float incoming_fule)
{
    Lunar_Fule = std::max(maximum_fule, Lunar_Fule+incoming_fule);
}

void LLEnv::Take_Action(Action_Info info)
{
    if (Lunar_Fule > 0)
    {
        Force_Applied_to_Right = info.RE_Active;
        Force_Applied_to_Left = info.LE_Active;
        Extra_Force_Applied_to_Right = info.RE_Boost;
        Extra_Force_Applied_to_Left = info.LE_Boost;
    }
}

void LLEnv::Step()
{
    // --- Apply Action ---
    Apply_Action();
    update_fule();
    // --- Step simulation ---
    world->stepSimulation(TimeStep, 32);

    Last_dist = New_dist;
    New_dist  = Calculate_Distance();

    // --- Resetting the Control Inputs
    last_RE_Active                  = Force_Applied_to_Right;
    last_LE_Active                  = Force_Applied_to_Left;
    last_RE_Boost                   = Extra_Force_Applied_to_Right;
    last_LE_Boost                   = Extra_Force_Applied_to_Left;
    Force_Applied_to_Right          = false;
    Force_Applied_to_Left           = false;
    Extra_Force_Applied_to_Right    = false;
    Extra_Force_Applied_to_Left     = false;
    Advanced                        = true;
}

void LLEnv::Get_Observation(Observation_Info& info)
{
    btTransform trans;
    All_Rect_Shapes_Info[LunrIdx].Blt_Body->getMotionState()->getWorldTransform(trans);
    btQuaternion rot = trans.getRotation();
    btScalar angleRad = rot.getAngle();
    if (rot.getAxis().getZ() < 0) angleRad = -angleRad;
    float angleDeg = angleRad * (180.0f / (float)M_PI);

    if (angleDeg > 180)  angleDeg -= 360;
    if (angleDeg < -180) angleDeg += 360;
    
    float Ang_val = fabsf(angleDeg);
    float Ang_sgn = 1.0f;
    if (angleDeg < 0) Ang_sgn = -1.0f;

    info.state.Lander_Pos_X             = trans.getOrigin().getX();
    info.state.Lander_Pos_Y             = trans.getOrigin().getY();
    info.state.Lander_LV_X              = All_Rect_Shapes_Info[LunrIdx].Blt_Body->getLinearVelocity().getX();
    info.state.Lander_LV_Y              = All_Rect_Shapes_Info[LunrIdx].Blt_Body->getLinearVelocity().getY();
    info.state.Lander_Ang_val           = Ang_val / 180.0f;
    info.state.Lander_Ang_sgn           = Ang_sgn;
    info.state.Lander_Ang_vel           = All_Rect_Shapes_Info[LunrIdx].Blt_Body->getAngularVelocity().getZ();
    //info.state.wind_speed_Direction_X   = 0.0f;
    //info.state.wind_speed_Direction_Y   = 0.0f;
    //info.state.Platform_Pos_X           = Platform_center_X;
    //info.state.Platform_Pos_Y           = Platform_Y;
    info.state.fule_left                = Lunar_Fule / maximum_fule * 100.0f;

    info.Done = is_Done(info);
    info.Reward = Claculate_Reward(info);


    // Scaling the positions 
    info.state.Lander_Pos_X = (trans.getOrigin().getX() - Window_Midle_X) / (Window_Midle_X);
    info.state.Lander_Pos_Y =  trans.getOrigin().getY()                   / (Window_Height / SCALE);
}

void LLEnv::Print_Observation_info(Observation_Info Oinfo, Action_Info Ainfo)
{
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout               <<  std::setw(6) << std::left << "Done:"                              << Oinfo.Done << " | ";
    std::cout               <<  std::setw(9) << std::left << "Action:"                            << Ainfo.RE_Active << Ainfo.LE_Active << Ainfo.RE_Boost << Ainfo.LE_Boost << " | ";
    std::cout << "Pos("     <<  std::setw(9) << Oinfo.state.Lander_Pos_X << ", " <<  std::setw(9) << Oinfo.state.Lander_Pos_Y << ") | ";
    std::cout << "Vel("     <<  std::setw(9) << Oinfo.state.Lander_LV_X << ", " <<   std::setw(9) << Oinfo.state.Lander_LV_Y << ") | ";
    std::cout << "Ang "     <<  std::setw(9) << Oinfo.state.Lander_Ang_sgn * Oinfo.state.Lander_Ang_val * 180.0f << " | ";
    std::cout << "Ang Vel " <<  std::setw(9) << Oinfo.state.Lander_Ang_vel << " | ";
    std::cout << "Fule "    <<  std::setw(9) << Oinfo.state.fule_left << " | ";
    std::cout << "Rew "     <<  std::setw(9) << Oinfo.Reward << std::endl;
}

void LLEnv::restart()
{
    ClearUp();
    Initialize();
}

void LLEnv::Set_Engine_Angle(float REng, float LEng)
{
    Right_Eng_Direction = REng;
    Left_Eng_Direction  = LEng;
}

bool LLEnv::Active_window_isOpen()
{
    if (!window.has_value())
        return false;
    return window->isOpen();
}

void LLEnv::Close_Active_Window()
{
    if (window.has_value())
        window->close();
}

void LLEnv::Disable_FPS_Limit()
{
    MFPS = 0;
    if (window.has_value())
        window->setFramerateLimit(0);
}

void LLEnv::Set_FPS_limit(int FPS)
{
    MFPS = FPS;
    if (window.has_value())
        window->setFramerateLimit(MFPS);
}

void LLEnv::Hide_Window()
{
    if (window.has_value())
        window->setVisible(false);
}

void LLEnv::Show_Window()
{
    if (window.has_value())
        window->setVisible(true);
}

void  LLEnv::Create_Window(std::string strx)
{
    if (!window.has_value())
    {
        this->str = strx;
        window.emplace(sf::VideoMode({ Window_Width, Window_Height }), this->str.data());
        window->setFramerateLimit(MFPS);
    }
}

void LLEnv::Clear_Window()
{
    // --- Draw ---
    if (window.has_value())
        window->clear(Color_Window_Background);      // dark gray-blue
}

void LLEnv::Polling_Window_Event()
{
    if (window.has_value())
    {
        if (Active_window_isOpen())
        {
            while (auto eventOpt = window->pollEvent())  // pollEvent() returns std::optional<sf::Event>
            {
                if (!eventOpt) continue;  // skip if no event

                sf::Event& event = *eventOpt;  // dereference the optional

                if (event.is<sf::Event::Closed>())  // window close event
                    Close_Active_Window();
            }
        }
    }
}

void LLEnv::draw_all()
{   
    if (window.has_value())
    {
        //  All_Rect_Shapes_Info;
        //  Draw all Rectangle Shapes
        //std::cout << All_Rect_Shapes_Info.size() << std::endl;
        for (auto& shape : All_Rect_Shapes_Info)
        {
            //std::cout << shape.name << std::endl;
            //std::cout << shape.SF_Shape.getOrigin().x   << ", " << shape.SF_Shape.getOrigin().y   << std::endl;
            if (!shape.is_static)
            {
                // Get lander transform & position/orientation
                btTransform trans;
                shape.Blt_Body->getMotionState()->getWorldTransform(trans);
                btVector3 bodyPos = trans.getOrigin();
                btQuaternion rot = trans.getRotation();
                // Convert Bullet quaternion to a single 2D rotation angle (degrees)
                // getAngle() returns magnitude, need sign from axis.z
                btScalar angleRad = rot.getAngle();
                if (rot.getAxis().getZ() < 0) angleRad = -angleRad;
                float angleDeg = angleRad * (180.0f / (float)M_PI);
                // compute world position of a child given its local Bullet offset macro
                sf::Vector2f p = True_Translation(shape.Blt_Offset, trans);
                shape.SF_Shape.setPosition(p);
                shape.SF_Shape.setRotation(sf::degrees(-angleDeg));
            }
            //std::cout << shape.SF_Shape.getPosition().x << ", " << shape.SF_Shape.getPosition().y << std::endl;
            window->draw(shape.SF_Shape);
            //window->display();
        }


        //  All_Circ_Shapes_Info;
        //  Draw all Circles 
        //std::cout << All_Circ_Shapes_Info.size() << std::endl;
        for (auto& shape : All_Circ_Shapes_Info)
        {
            //std::cout << shape.name << std::endl;
            if (!shape.is_static)
            {
                // Get lander transform & position/orientation
                btTransform trans;
                shape.Blt_Body->getMotionState()->getWorldTransform(trans);
                btVector3 bodyPos = trans.getOrigin();
                // compute world position of a child given its local Bullet offset macro
                sf::Vector2f p = True_Translation(shape.Blt_Offset, trans);
                shape.SF_Shape.setPosition(p);
            }
            window->draw(shape.SF_Shape);
            //window->display();
        }



        //  All_Cnvx_Shapes_Info;
        //  Draw all Convex Shapes 
        int i = -1;
        //std::cout << All_Cnvx_Shapes_Info.size() << std::endl;
        for (auto& shape : All_Cnvx_Shapes_Info)
        {
            i++;
            //std::cout << shape.name << std::endl;
            //std::cout << shape.SF_Shape.getOrigin().x   << ", " << shape.SF_Shape.getOrigin().y   << std::endl;
            if (!shape.is_static)
            {
                // Get lander transform & position/orientation
                btTransform trans;
                shape.Blt_Body->getMotionState()->getWorldTransform(trans);
                btVector3 bodyPos = trans.getOrigin();
                btQuaternion rot = trans.getRotation();
                // Convert Bullet quaternion to a single 2D rotation angle (degrees)
                // getAngle() returns magnitude, need sign from axis.z
                btScalar angleRad = rot.getAngle();
                if (rot.getAxis().getZ() < 0) angleRad = -angleRad;
                float angleDeg = angleRad * (180.0f / (float)M_PI);
                // compute world position of a child given its local Bullet offset macro
                sf::Vector2f p = True_Translation(shape.Blt_Offset, trans);
                shape.SF_Shape.setPosition(p);
                if (i == REngIdx) angleDeg -= Right_Eng_Direction;
                if (i == LEngIdx) angleDeg -= Left_Eng_Direction;
                shape.SF_Shape.setRotation(sf::degrees(-angleDeg));
            }
            //std::cout << shape.SF_Shape.getPointCount() << std::endl;
            //std::cout << "P0:\t" << shape.SF_Shape.getPoint(0).x << ", " << shape.SF_Shape.getPoint(0).x << std::endl;
            //std::cout << "P1:\t" << shape.SF_Shape.getPoint(1).x << ", " << shape.SF_Shape.getPoint(1).x << std::endl;
            //std::cout << "P2:\t" << shape.SF_Shape.getPoint(2).x << ", " << shape.SF_Shape.getPoint(2).x << std::endl;
            //std::cout << shape.SF_Shape.getPosition().x << ", " << shape.SF_Shape.getPosition().y << std::endl;
            window->draw(shape.SF_Shape);
            //window->display();
        }


        draw_vapor();
    }
}

void LLEnv::Render()
{
    if (window.has_value())
        window->display();
}

void LLEnv::Create_Video(std::string strx)
{
    if (!writer.has_value())
    {
        this->str = strx;
        writer.emplace(cv::VideoWriter(strx.data(), cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 60, cv::Size(Window_Width, Window_Height)));
    }
}

void LLEnv::Add_a_Frame()
{
    if (!writer.has_value() || !writer->isOpened()) return;

    // Capture SFML window
    sf::Texture texture(sf::Vector2u(window->getSize().x, window->getSize().y));
    texture.update(window.value());
    sf::Image img = texture.copyToImage();

    // Convert SFML image to OpenCV Mat (BGR)
    const unsigned char* pixels = img.getPixelsPtr();
    cv::Mat frame(window->getSize().y, window->getSize().x, CV_8UC4, (void*)pixels);
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_RGBA2BGR);

    writer->write(bgr);
}

void LLEnv::Release_video()
{
    if (writer.has_value())
        if (writer->isOpened())
            writer->release();
    writer.reset();
}

void LLEnv::draw_vapor()
{
    //  Location of Lunar Lander
    btTransform trans;
    All_Rect_Shapes_Info[LunrIdx].Blt_Body->getMotionState()->getWorldTransform(trans);


    //
    //  Right
    //if (++RVpCntr > VaprDec)
    //{
    //    RVpCntr = 0;
        //  Local direction of exhaust (in lander’s local space)
        btVector3 REngine_Pos = trans * -btVector3(LL_Blt_Origin_Right_Eng);
        btVector3 REngine_Dir = trans.getBasis() * btVector3(std::sin(Right_Eng_Direction / 180.0f * (float)M_PI), -std::cos(Right_Eng_Direction / 180.0f * (float)M_PI), 0);
        REngine_Dir.normalize();
        REngine_Dir = Vapor_ejection_speed_base * REngine_Dir;
        //  fixed location for testing
        REV->setEmitter(REngine_Pos);
        REV->setDirection(REngine_Dir);
        //  Emmiting
        REV->emit(Vapor_cnt * last_RE_Active, 1 + last_RE_Boost);
        //  Update Lifetimes
        REV->update(TimeStep);
        //  Draw
        REV->draw(*window);
    //}

    //
    //  Left
    //if (++LVpCntr > VaprDec)
    //{
    //    LVpCntr = 0;
        //  Local direction of exhaust (in lander’s local space)
        btVector3 LEngine_Pos = trans * -btVector3(LL_Blt_Origin_Left_Eng);
        btVector3 LEngine_Dir = trans.getBasis() * btVector3(std::sin(Left_Eng_Direction / 180.0f * (float)M_PI), -std::cos(Left_Eng_Direction / 180.0f * (float)M_PI), 0);
        LEngine_Dir.normalize();
        LEngine_Dir = Vapor_ejection_speed_base * LEngine_Dir;
        //  fixed location for testing
        LEV->setEmitter(LEngine_Pos);
        LEV->setDirection(LEngine_Dir);
        //  Emmiting
        LEV->emit(Vapor_cnt * last_LE_Active, 1 + last_LE_Boost);
        //  Update Lifetimes
        LEV->update(TimeStep);
        //  Draw
        LEV->draw(*window);
    //}
}

void LLEnv::Apply_Action()
{
    if (Force_Applied_to_Right)  ApplyEngineThrust(LL_Blt_Origin_Right_Eng, Right_Eng_Direction, 1.0 + Extra_Force_Applied_to_Right);
    if (Force_Applied_to_Left)   ApplyEngineThrust(LL_Blt_Origin_Left_Eng,  Left_Eng_Direction,  1.0 + Extra_Force_Applied_to_Left);
}

void LLEnv::update_fule()
{   
    float Consummed(Base_fule_consumption);
    Consummed += Force_Applied_to_Right       * (normal_fule_consumption + Extra_Force_Applied_to_Right * extra_fule_consumption);
    Consummed += Force_Applied_to_Left        * (normal_fule_consumption + Extra_Force_Applied_to_Left  * extra_fule_consumption);

    Lunar_Fule -= Consummed;
    if (Lunar_Fule <= 0) Lunar_Fule = 0;
}

void LLEnv::ClearUp()
{
    // --- 0. Keep track of deleted bodies to avoid double deletion ---
    std::set<btRigidBody*> deletedBodies;

    auto safe_delete_bodies = [this, &deletedBodies](auto& vec)
        {
            for (auto& info : vec)
            {
                btRigidBody* body = info.Blt_Body;
                if (body && deletedBodies.find(body) == deletedBodies.end())
                {
                    if (world) world->removeRigidBody(body);

                    if (body->getMotionState())
                    {
                        delete body->getMotionState();
                    }

                    delete body;
                    deletedBodies.insert(body);
                }
                info.Blt_Body = nullptr;
            }
            vec.clear();
        };

    // --- 1. Delete all shapes safely ---
    safe_delete_bodies(All_Rect_Shapes_Info);
    safe_delete_bodies(All_Circ_Shapes_Info);
    safe_delete_bodies(All_Cnvx_Shapes_Info);

    // --- 2. Delete Lunar Lander parts ---
    if (LunLander && deletedBodies.find(LunLander) == deletedBodies.end())
    {
        if (world) world->removeRigidBody(LunLander);
        if (motionState) delete motionState;
        delete LunLander;
        motionState = nullptr;
        LunLander = nullptr;
    }

    delete LL_compound; LL_compound = nullptr;
    delete LL_body;     LL_body = nullptr;
    delete LL_Arm;      LL_Arm = nullptr;
    delete LL_leg;      LL_leg = nullptr;
    delete LL_Eng;      LL_Eng = nullptr;

    // --- 3. Delete mountain ---
    if (mountainBody && deletedBodies.find(mountainBody) == deletedBodies.end())
    {
        if (world) world->removeRigidBody(mountainBody);
        if (mountainMotionState) delete mountainMotionState;
        delete mountainBody;
        delete mountainShape;
        delete mountainMesh;
        mountainBody = nullptr;
        mountainMotionState = nullptr;
        mountainShape = nullptr;
        mountainMesh = nullptr;
    }

    // --- 4. Delete engine vapors ---
    delete REV; REV = nullptr;
    delete LEV; LEV = nullptr;

    // --- 5. Delete Bullet world helpers ---
    for (auto& shape : Shapes) delete shape;
    Shapes.clear();

    delete world;      world = nullptr;
    delete solver;     solver = nullptr;
    delete broadphase; broadphase = nullptr;
    delete dispatcher; dispatcher = nullptr;
    delete collisionConfig; collisionConfig = nullptr;

    // --- 6. Close SFML window safely ---
    if (window.has_value())
    {
        window->close();
        window.reset();
    }
}

void LLEnv::Initialize()
{
    // --- 1. Reintialize
    Lunar_Fule = 0;

    LEngIdx = -1;
    REngIdx = -1;
    LunrIdx = -1;
    Force_Applied_to_Right          = false;
    Force_Applied_to_Left           = false;
    Extra_Force_Applied_to_Right    = false;
    Extra_Force_Applied_to_Left     = false;
    last_RE_Active  = false;
    last_LE_Active  = false;
    last_RE_Boost   = false;
    last_LE_Boost   = false;
    Advanced        = true;

    Let_be_Universe();
    Let_be_Gravity({ Gravity_X , Gravity_Y, Gravity_Z});
    Let_be_a_Ground();
    //Let_be_a_Rectangle(Body_Half_Width, Body_Half_Height, Body_Half_Depth, Body_Pos_X, Body_Pos_Y, Body_Pos_Z, Body_mass, Color_Box1);
    //Let_be_a_Rectangle(Body_Half_Width, Body_Half_Height, Body_Half_Depth, Body_Pos_X + 2.0f, Body_Pos_Y + 2.0f, Body_Pos_Z, Body_mass, Color_Box2);
    //Let_be_a_Rectangle(Body_Half_Width, Body_Half_Height, Body_Half_Depth, Body_Pos_X + 4.0f, Body_Pos_Y + 4.0f, Body_Pos_Z, Body_mass, Color_Ground);
    //Let_be_a_Rectangle(Body_Half_Width, Body_Half_Height, Body_Half_Depth, Body_Pos_X + 6.0f, Body_Pos_Y + 6.0f, Body_Pos_Z, Body_mass, Color_Flag_Pole);
    //Let_be_a_Circle(Circle_Radius, Circle_Pos_X, Circle_Pos_Y, Circle_Pos_Z, Circle_mass, Color_Box2);
    Let_be_a_Mountain();
    Let_be_a_Platform_Flags();


    float tmp = sqrt(floatDist(gen));
    float sgn = (floatDist(gen) > 0.5) ? 1.0f : -1.0f;
    float LLx = (Window_Width / SCALE / 2) + sgn * tmp * (Window_Width / SCALE / 2);
    float Bse = Platform_Y + Minimum_Respawn_Attitude + abs(LLx - Platform_center_X) * Mount_max_slope;
    float LLy = Bse + (Window_Height/ SCALE - Bse) * floatDist(gen);

    Let_be_a_LunarLander(LLx, LLy, LL_Pos_Z, LL_mass, Color_LunarLander);
    Let_be_a_Engine_Vapor();

    New_dist = Calculate_Distance();
    Last_dist = New_dist;
    Next_Goal = floor(New_dist / Check_Point_Distances) * Check_Point_Distances;


    Observation_Info info;
    Get_Observation(info);
}

bool LLEnv::is_Done(Observation_Info info)
{
    bool out_of_scope   = is_Out_of_Scope(info);
    bool out_of_fule    = (Lunar_Fule == 0);
    bool contact        = is_Make_Contact_with_Platform(info);
    bool crashed        = is_Make_Contact_with_Mountain(info);
    return out_of_scope || contact || crashed || out_of_fule;
}

bool LLEnv::is_Out_of_Scope(Observation_Info info)
{
    float dx = info.state.Lander_Pos_X - Platform_center_X;
    float dy = info.state.Lander_Pos_Y - Platform_Y;

    return sqrt(dx * dx + dy * dy) >= Max_Distance_Allowed;
}

bool LLEnv::is_Make_Contact_with_Platform(Observation_Info info)
{
    int numManifolds = world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i)
    {
        btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        if ((obA == LunLander && obB == mountainBody) ||
            (obB == LunLander && obA == mountainBody))
        {
            int numContacts = contactManifold->getNumContacts();
            if (numContacts > 0)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint(0);
                btVector3 ptPos = pt.getPositionWorldOnA();

                float landerX = info.state.Lander_Pos_X;
                float landerY = info.state.Lander_Pos_Y;
                
                bool onX = (landerX >= (Platform_X + (LL_Unit * LL_Body_Half_Size))) && (landerX <= (Platform_X + Platform_Length - (LL_Unit * LL_Body_Half_Size)));
                
                return onX;
                //contactPos = toSFML(ptPos); // converts Bullet vec to SFML coords
                //return true;
            }
        }
    }
    return false;
}

bool LLEnv::is_Make_Contact_with_Mountain(Observation_Info info)
{
    int numManifolds = world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i)
    {
        btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        if ((obA == LunLander && obB == mountainBody) ||
            (obB == LunLander && obA == mountainBody))
        {
            int numContacts = contactManifold->getNumContacts();
            if (numContacts > 0)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint(0);
                btVector3 ptPos = pt.getPositionWorldOnA();

                float landerX = info.state.Lander_Pos_X;
                float landerY = info.state.Lander_Pos_Y;

                bool onX = (landerX <= (Platform_X + (LL_Unit * LL_Body_Half_Size))) || (landerX >= (Platform_X + Platform_Length - (LL_Unit * LL_Body_Half_Size)));

                return onX;
                //contactPos = toSFML(ptPos); // converts Bullet vec to SFML coords
                //return true;
            }
        }
    }
    return false;
}

float LLEnv::Calculate_Distance()
{
    btTransform trans;
    All_Rect_Shapes_Info[LunrIdx].Blt_Body->getMotionState()->getWorldTransform(trans);
    float dx = trans.getOrigin().getX() - Platform_center_X;
    float dy = trans.getOrigin().getY() - Platform_Y;
    return sqrt(dx * dx + dy * dy);
}

float LLEnv::Claculate_Reward(Observation_Info New_info)
{
    float total_reward(-0.5f);

    // version 5
    auto curr = New_info;
    auto prev = Old_info;
    // --- Normalizers (same as before) ---
    const float Dnorm = 10.0f;
    const float Vnorm = 2.5f;
    const float Thnorm = 6.0f;      // degrees
    const float Omeganorm = 5.0f;

    // --- Weights (same defaults) ---
    const float wp = 0.5f;
    const float alphap = 3.0f;
    const float wv = 0.15f;
    const float alphav = 2.0f;
    const float wtheta = 0.075f;
    const float womega = 0.05f;
    const float wt = -0.02f;         // time penalty
    const float gamma = 0.99f;       // discount
    const float R_land = 500.0f;
    const float R_crash = 500.0f;
    const float wFuelTerminal = 50.0f;
    const float ws = 120.0f;


    // --- Extract and normalize current state ---
    float dx = curr.state.Lander_Pos_X - Platform_center_X;
    float dy = curr.state.Lander_Pos_Y - (Platform_Y + LL_Unit * (LL_Body_Half_Size + 2 * LL_Leg_half_Height));
    float dist = sqrtf(dx * dx + dy * dy);
    float d_norm = dist;

    float vx = curr.state.Lander_LV_X;
    float vy = curr.state.Lander_LV_Y;
    float v_norm = sqrtf(vx * vx + vy * vy) / Vnorm;

    float ang = fabsf(curr.state.Lander_Ang_val * 180.0f);
    float ang_norm = ang / Thnorm;

    float angv_norm = fabsf(curr.state.Lander_Ang_vel) / Omeganorm;

    // --- Potential shaping ---
    float phi_now = -d_norm;
    float prev_dx = prev.state.Lander_Pos_X - Platform_center_X;
    float prev_dy = prev.state.Lander_Pos_Y - (Platform_Y + LL_Unit * (LL_Body_Half_Size + 2 * LL_Leg_half_Height));
    float prev_dist = sqrtf(prev_dx * prev_dx + prev_dy * prev_dy);
    float phi_prev = -(prev_dist);
    //float shape_gamma = expf((-Gravity_Y * TimeStep) / (15.0f + 2.0f * fabsf(dy)));
    float shape_gamma = 1.0f;

    float shape = ws * (shape_gamma * phi_now - phi_prev);



    float dense = shape * shape * shape * (1 + 4 * (shape <= 0));

    // --- Smooth dense terms ---
    float vel_term = wv     * (expf(v_norm)   - 1);
    float ang_term = wtheta * (expf(ang_norm) - 1);
    float dst_coef = 1 / (dist + 0.25f);
    
    dense = dense - dst_coef * (vel_term + ang_term);

    //std::cout << 
    //    std::setw(8) << phi_now << "\t" << 
    //    std::setw(8) << phi_prev << "\t" << 
    //    std::setw(8) << shape << "\t" << 
    //    std::setw(8) << pos_term << "\t" << 
    //    std::setw(8) << vel_term << "\t" << 
    //    std::setw(8) << angle_term << std::endl;


    // --- Terminal detection using contact + conditions ---
    bool has_contact = is_Make_Contact_with_Platform(curr);  // your flag, boolean
    // Conditions for soft landing
    //bool near_x = fabsf(dx) < Platform_Length/2;
    //bool near_y = fabsf(dy) < 1.0f;
    //bool soft_vy = fabsf(curr.state.Lander_LV_Y) < 1.0f;
    //bool soft_vx = fabsf(curr.state.Lander_LV_X) < 1.5f;
    //bool upright = fabsf(curr.state.Lander_Ang_val*180) < 10.0f;

    bool landed = has_contact;

    bool crashed = false;
    // define crash: e.g. large speed or angle or out-of-bounds
    if (//fabsf(curr.state.Lander_LV_Y) > 3.5f    ||
        //fabsf(curr.state.Lander_Ang_val*180)  > 55.0f   ||
        is_Make_Contact_with_Mountain(curr)     ||
        is_Out_of_Scope(curr))
    {
        crashed = true;
    }

    float term_reward = 0.1f;

    if (landed)
    {
        float fuel_frac = curr.state.fule_left / 100.0f;
        term_reward = +R_land + fuel_frac;
    }
    else if (crashed)
    {
        term_reward = -R_crash;
    }

    float total = dense + term_reward;

    // clamp
    if (total > 1000.0f) total =1000.0f;
    if (total < -1000.0f) total = -1000.0f;

    // optionally set a flag to notify training loop if 'landed' or 'crashed' (curr.Done true)

    total_reward += total;



    // version 4
    /*
    float total_reward(0);
    total_reward -= Reward_Pen_4_Base_Actions;
    if (!New_info.Done)
    {
        if (Calculate_Distance() <= Next_Goal)
        {
            total_reward    +=  Reward_Val_4_Change_Distance;
            Next_Goal       -=  Check_Point_Distances;
        }
        if (Calculate_Distance() >= (Next_Goal + Check_Point_Distances))
        {
            total_reward    -=  10 * Reward_Val_4_Change_Distance;
            Next_Goal       += Check_Point_Distances;
        }
    }
    else
    {
        if (is_Out_of_Scope(New_info))
            total_reward    -=  Reward_Pen_4_Out_of_Scope;
        if (is_Make_Contact_with_Mountain(New_info))
            total_reward    -=  Reward_Pen_4_Make_UnSuccessful_Contact;
        if (is_Make_Contact_with_Platform(New_info))
        {
            total_reward += Reward_Val_4_Make_Successful_Contact;

            float dx    = (New_info.state.Lander_Pos_X - Platform_center_X);
            float pang  = New_info.state.Lander_Ang / 180.0f;
            float pvx   = Old_info.state.Lander_LV_X;
            float pvy   = Old_info.state.Lander_LV_Y;
            float pavel = Old_info.state.Lander_Ang_vel;
            float pvel  = sqrtf(pvx * pvx + pvy * pvy)/5;

            float fuel_Reward   = Reward_Val_4_Fule_Left    * (New_info.state.fule_left / 100.0f);
            float angle_penalty = Reward_Pen_4_Off_Balance  * fabs(pang);
            float dist_penalty  = Reward_Pen_4_Off_center   * (1 - expf(-1.0f * abs(dx)));
            float vel_penalty   = Reward_Pen_4_Lin_Velocity * (1 - expf(-1.0f * pvel));
            float avel_penalty  = Reward_Pen_4_Ang_Velocity * (1 - expf(-1.0f * abs(pavel)));

            total_reward += fuel_Reward;
            total_reward -= angle_penalty;
            total_reward -= dist_penalty;
            total_reward -= vel_penalty;
            total_reward -= avel_penalty;
        }
    }
    total_reward /= 10;
    */



    // Version 3
    /*
    auto curr  = New_info;
    auto prev  = Old_info;
    Old_info   = New_info;
    float dx   = (curr.state.Lander_Pos_X - Platform_center_X)  / (Ground_Half_Width); // normalized position
    float dy   = (curr.state.Lander_Pos_Y - Platform_Y)         / (Window_Height / SCALE - Platform_Y);
    float vx   = curr.state.Lander_LV_X / 5.0f;
    float vy   = curr.state.Lander_LV_Y / 5.0f;
    float ang  = curr.state.Lander_Ang / 180.0f;       // normalize to [-1, +1]
    float angv = curr.state.Lander_Ang_vel / 5.0f;    // normalize angular velocity
    float fuel = curr.state.fule_left / 100.0f;       // 0–1

    // --- Previous normalized states for potential shaping ---
    float prev_dx = (prev.state.Lander_Pos_X - Platform_center_X)   / (Ground_Half_Width); // normalized position
    float prev_dy = (prev.state.Lander_Pos_Y - Platform_Y)          / (Window_Height / SCALE - Platform_Y);

    // --- Distance and velocity magnitudes ---
    float dist_now = sqrtf(dx * dx + dy * dy);
    float dist_prev = sqrtf(prev_dx * prev_dx + prev_dy * prev_dy);
    float vel_mag = sqrtf(vx * vx + vy * vy);

    // --- Potential-based shaping term ---
    float gamma = 0.99f;
    float potential = -dist_now;
    float prev_potential = -dist_prev;
    float potential_shaping = gamma * potential - prev_potential;

    // --- Smooth penalties & bonuses ---
    float dist_reward = 0.4f * expf(-3.0f * dist_now);          // strong near center
    float vel_reward = 0.3f * expf(-2.0f * vel_mag);           // smoother near low velocity
    float angle_reward = 0.2f * expf(-fabsf(ang));               // upright posture
    float fuel_bonus = 20.0f * fuel;                           // encourage efficiency
    float hot_lander_penalty = -10 * (fabs(curr.state.Lander_LV_Y) > 5);

    // --- Composite dense reward ---
    float dense_reward = potential_shaping + dist_reward + vel_reward + angle_reward + hot_lander_penalty;

    // --- Terminal events ---
    bool near_platform = (fabs(curr.state.Lander_Pos_X - curr.state.Platform_Pos_X) < 0.5f);
    bool soft_descent  = (fabs(curr.state.Lander_LV_Y) < 0.5f);
    bool upright       = (fabs(curr.state.Lander_Ang) < 10.0f);
    bool close_to_pad  = (fabs(curr.state.Lander_Pos_Y - curr.state.Platform_Pos_Y) < 1.0f);

    bool landed = near_platform && soft_descent && upright && close_to_pad;
    bool crashed = fabs(curr.state.Lander_Ang) > 45.0f;

    float terminal_reward = 0.0f;
    if (is_Make_Contact_with_Platform(curr))
        terminal_reward += 100.0f + fuel_bonus;  // successful gentle landing
    if (is_Make_Contact_with_Mountain(curr))
        terminal_reward -= 100.0f;
    if (crashed)
        terminal_reward -= 100.0f;  // explosion or tip-over

    // --- Step penalty to motivate quick landing ---
    float time_penalty = -0.01f;

    // --- Combine everything ---
    float reward = dense_reward + terminal_reward + time_penalty;

    // --- Clamp reward to avoid explosion ---
    reward = std::clamp(reward, -150.0f, 150.0f);


    return reward;
    */



    //  Version 2
    /*
    auto info  = New_info;
    float dx   = (info.state.Lander_Pos_X - Platform_center_X);
    float dy   = (info.state.Lander_Pos_Y - Platform_Pos_Y);
    float vx   = info.state.Lander_LV_X;
    float vy   = info.state.Lander_LV_Y;
    float ang  = info.state.Lander_Ang * M_PI / 180.0f; // convert to radians
    float angv = info.state.Lander_Ang_vel;
    float fuel = info.state.fule_left / 100.0f;         // normalized 0–1

    // Distance & velocity terms (negative cost)
    float dist_penalty  = -0.5f * sqrt(dx * dx + dy * dy);
    float vel_penalty   = -0.3f * sqrt(vx * vx + vy * vy);
    float angle_penalty = -0.2f * (fabs(ang) + 0.1f * fabs(angv));

    // Encourage fuel efficiency
    float fuel_bonus = +0.1f * fuel;

    // Reward being close to target area
    float center_bonus = expf(-fabs(dx)) * 0.5f;

    // Base shaping reward
    float reward = dist_penalty + vel_penalty + angle_penalty + fuel_bonus + center_bonus;

    bool crashed = (fabs(vy) > 2.0f || fabs(ang) > 45.0f);

    // Terminal rewards
    if (info.Done)
    {
        if (is_make_contact(info))       reward += +100.0f;   // perfect soft landing
        else if (crashed) reward += -100.0f;   // crash or tip over
    }

    // Optional small step penalty to speed up landing
    reward -= 0.01f;

    return reward;
    */
    
    
    
    //  Version 1
    /*
    float dx = abs(info.state.Lander_Pos_X - Platform_center_X);
    float py = info.state.Lander_Pos_Y;
    
    
    float reward(0.0f);
    if (!info.Done)
    {
        if (py < (dx * (dx > Platform_Length / 2) * Mount_max_slope + Platform_Y + (dx > Platform_Length / 2) * 0.1f))
            reward -= Reward_Pen_4_Below_Mountain_Slope;
        else
            reward += Reward_Val_4_Change_Distance * (Last_dist - New_dist);
    }
    else
    {
        if (is_Out_of_Scope(info))
            reward -= Reward_Pen_4_Out_of_Scope;
        else
        {
            if (!is_make_contact(info))
                reward -= Reward_Pen_4_Make_UnSuccessful_Contact;
            else
            {
                reward += Reward_Val_4_Make_Successful_Contact;
                reward += Reward_Val_4_Fule_Left * info.state.fule_left;
                reward -= Reward_Pen_4_Off_center * dx;
                reward -= Reward_Pen_4_Rotation * abs(info.state.Lander_Ang / 180.0f);
                reward -= Reward_Pen_4_Lin_Velocity * sqrt(pow(info.state.Lander_LV_X, 2) + pow(info.state.Lander_LV_Y, 2));
                reward -= Reward_Pen_4_Ang_Velocity * abs(info.state.Lander_Ang_vel);
            }
        }
    }
    
    return reward;
    */



    if (Advanced)
    {
        Old_info = New_info;
        Advanced = false;
    }

    return total_reward;
}

float LLEnv::clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}










void LLEnv::Make_a_Rectangle(float Hw, float Hh, Rect_Shape_Info& info, sf::Color color)
{
    info.SF_Shape = sf::RectangleShape(sf::Vector2f(2 * Hw * SCALE, 2 * Hh * SCALE));
    info.SF_Shape.setOrigin({ Hw * SCALE, Hh * SCALE });
    info.SF_Shape.setFillColor(color);
}

void LLEnv::Make_a_Circle(float Radius, Circ_Shape_Info& info, sf::Color color)
{
    info.SF_Shape = sf::CircleShape(Radius * SCALE);
    info.SF_Shape.setOrigin({ Radius * SCALE, Radius * SCALE });
    info.SF_Shape.setFillColor(color);
}

void LLEnv::Make_a_Cnvx(std::vector<sf::Vector2f> nodes, Cnvx_Shape_Info& info, sf::Color color)
{
    sf::ConvexShape this_shape;
    this_shape.setPointCount(nodes.size());
    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        this_shape.setPoint(i, nodes.at(i));
    }
    this_shape.setOrigin({ 0, 0 });
    this_shape.setFillColor(color);
    info.SF_Shape = this_shape;
}














void LLEnv::Let_be_Universe()
{
    // --- Bullet Setup ---
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    world->getSolverInfo().m_numIterations = Physic_solver_iteration;
    world->getDispatchInfo().m_allowedCcdPenetration = Physic_max_allowed_penetration;
}

void LLEnv::Let_be_Gravity(btVector3 Gravity)
{
    world->setGravity(Gravity);          // Y-down
}

void LLEnv::Let_be_a_Ground()
{
    // --- Ground ---
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* groundMotion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, Ground_Pos_Y, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundInfo(0, groundMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(groundInfo);
    groundBody->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    groundBody->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    groundBody->setRestitution(Physic_Ground_Restitution);
    world->addRigidBody(groundBody);

    Shapes.push_back(groundShape);

    Rect_Shape_Info info;
    info.name = "Ground";
    info.is_static = true;
    info.Blt_Body = groundBody;
    info.Blt_Offset = { 0, 0, 0 };
    Make_a_Rectangle(Ground_Half_Width, Ground_Half_Height, info, Color_Ground);
    info.SF_Shape.setPosition(toSFML({ Ground_Half_Width, Ground_Half_Height, 0}));
    All_Rect_Shapes_Info.push_back(info);
}

void LLEnv::Let_be_a_Rectangle(float Hw, float Hh, float Hd, float Px, float Py, float Pz, float mass, sf::Color color)
{
    // --- Body (box) ---
    btCollisionShape* bodyShape = new btBoxShape(btVector3(Hw, Hh, Hd)); // 1m × 1m x 1m
    btDefaultMotionState* bodyMotion =
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(Px, Py, Pz)));
    btScalar bodyMass = mass;
    btVector3 bodyInertia(0, 0, 0);
    bodyShape->calculateLocalInertia(bodyMass, bodyInertia);
    btRigidBody::btRigidBodyConstructionInfo bodyInfo(bodyMass, bodyMotion, bodyShape, bodyInertia);
    btRigidBody* body = new btRigidBody(bodyInfo);
    body->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    body->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    body->setRestitution(Physic_Boxes_Restitution);
    world->addRigidBody(body);

    Shapes.push_back(bodyShape);

    Rect_Shape_Info info;
    info.name = "Let_be_a_Rectangle";
    info.is_static = false;
    info.Blt_Body = body;
    info.Blt_Offset = { 0, 0, 0 };
    Make_a_Rectangle(Hw, Hh, info, color);
    All_Rect_Shapes_Info.push_back(info);
}

void LLEnv::Let_be_a_Circle(float Radius, float Px, float Py, float Pz, float mass, sf::Color color)
{
    // --- Circle (ball) ---
    btCollisionShape* circleShape = new btSphereShape(Radius);  // keep units consistent with SCALE
    btDefaultMotionState* circleMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(Px, Py, Pz)));
    btScalar circleMass = mass;
    btVector3 circleInertia(0, 0, 0);
    circleShape->calculateLocalInertia(circleMass, circleInertia);
    btRigidBody::btRigidBodyConstructionInfo circleRigidBodyCI(circleMass, circleMotionState, circleShape, circleInertia);
    btRigidBody* Circ = new btRigidBody(circleRigidBodyCI);
    // lock Z movement and rotation -> pure 2D
    Circ->setRestitution(Physic_Circles_Restitution);
    Circ->setLinearFactor(btVector3(1, 1, 0));
    Circ->setAngularFactor(btVector3(0, 0, 1));
    world->addRigidBody(Circ);

    Shapes.push_back(circleShape);

    Circ_Shape_Info info;
    info.name = "Let_be_a_Circle";
    info.is_static = false;
    info.Blt_Body = Circ;
    info.Blt_Offset = { 0, 0, 0 };
    Make_a_Circle(Radius, info, color);
    All_Circ_Shapes_Info.push_back(info);
}

void LLEnv::Let_be_a_Mountain()
{
    // --- Mountain Mesh ---
    // Create triangle mesh
    mountainMesh = new btTriangleMesh();
    bool before(true);
    bool after(false);
    float x_base(0);
    float y_base(Ground_Pos_Y + Mount_Ave_Height + Mount_Max_DeltaY * normDist(gen));
    float x_next(0);
    float y_next(0);
    float x_prev(0);
    float y_prev(0);
    while (x_next <= Mount_length)
    {
        float random_Height = normDist(gen);
        float random_length = floatDist(gen);

        if (before || after)
        {
            float diff = std::abs(Platform_center_X - x_next);
            x_next = x_base + random_length * Mount_Max_DeltaX;
            y_next = std::max(std::min(Platform_Y + (diff * Mount_max_slope), y_base + random_Height * Mount_Max_DeltaY), Ground_Pos_Y);
        }
        else
        {
            x_next = Platform_X + Platform_Length;
            after = true;
        }

        if ((x_next >= Platform_X) && before)
        {
            x_next = Platform_X;
            y_next = Platform_Y;
            before = false;
        }

        btVector3 T1N1(x_base, Ground_Pos_Y, 0);
        btVector3 T1N2(x_base, y_base, 0);
        btVector3 T1N3(x_next, y_next, 0);
        btVector3 T2N1(x_base, Ground_Pos_Y, 0);
        btVector3 T2N2(x_next, y_next, 0);
        btVector3 T2N3(x_next, Ground_Pos_Y, 0);

        mountainMesh->addTriangle(T1N1, T1N2, T1N3);
        mountainMesh->addTriangle(T2N1, T2N2, T2N3);

        x_prev = x_base;
        y_prev = y_base;
        x_base = x_next;
        y_base = y_next;


        Cnvx_Shape_Info	T1info;
        T1info.name = "Mountain";
        T1info.is_static = true;
        std::vector<sf::Vector2f> T1nodes = { toSFML(T1N1), toSFML(T1N2), toSFML(T1N3) };
        Make_a_Cnvx(T1nodes, T1info, Color_Mountain);
        All_Cnvx_Shapes_Info.push_back(T1info);

        Cnvx_Shape_Info	T2info;
        T2info.name = "Mountain";
        T2info.is_static = true;
        std::vector<sf::Vector2f> T2nodes = { toSFML(T2N1), toSFML(T2N2), toSFML(T2N3) };
        Make_a_Cnvx(T2nodes, T2info, Color_Mountain);
        All_Cnvx_Shapes_Info.push_back(T2info);
    }

    // Create static concave shape
    mountainShape = new btBvhTriangleMeshShape(mountainMesh, true);
    // No mass -> static
    mountainMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))
    );
    btRigidBody::btRigidBodyConstructionInfo mountainCI(0.0f, mountainMotionState, mountainShape, btVector3(0, 0, 0));
    mountainBody = new btRigidBody(mountainCI);
    mountainBody->setRestitution(Physic_Mountain_Restitution);
    world->addRigidBody(mountainBody);
}

void LLEnv::Let_be_a_LunarLander(float Px, float Py, float Pz, float mass, sf::Color color)
{
    // --- Create a compound shape ---
    LL_compound = new btCompoundShape();
    // Green body
    LL_body = new btBoxShape(btVector3(LL_Unit * LL_Body_Half_Size, LL_Unit * LL_Body_Half_Size, LL_Depth));
    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(LL_Blt_Origin_body);
    LL_compound->addChildShape(t1, LL_body);
    // Arm
    // Right Arm
    btBoxShape* LL_Arm = new btBoxShape(btVector3(LL_Unit * LL_Arm_Half_Width, LL_Unit * LL_Arm_Half_Height, LL_Depth));
    btTransform t2;
    t2.setIdentity();
    t2.setOrigin(LL_Blt_Origin_Right_Arm);
    LL_compound->addChildShape(t2, LL_Arm);
    // Left Arm
    btTransform t3;
    t3.setIdentity();
    t3.setOrigin(LL_Blt_Origin_Left_Arm);
    LL_compound->addChildShape(t3, LL_Arm);
    // legs
    // Right Leg
    LL_leg = new btBoxShape(btVector3(LL_Unit * LL_Leg_Half_Width, LL_Unit * LL_Leg_half_Height, LL_Depth));
    btTransform t4;
    t4.setIdentity();
    t4.setOrigin(LL_Blt_Origin_Right_Leg);
    LL_compound->addChildShape(t4, LL_leg);
    // Left Leg
    btTransform t5;
    t5.setIdentity();
    t5.setOrigin(LL_Blt_Origin_Left_Leg);
    LL_compound->addChildShape(t5, LL_leg);
    // Engine
    // Right Engine
    LL_Eng = new btBoxShape(btVector3(LL_Unit * LL_Eng_Half_Length, LL_Unit * LL_Eng_Half_Length, LL_Depth));
    btTransform t6;
    t6.setIdentity();
    t6.setOrigin(LL_Blt_Origin_Right_Eng);
    t6.getBasis().setEulerZYX(0, 0, SIMD_PI * Right_Eng_Direction / 180);  // Rotate -30° around Z axis
    LL_compound->addChildShape(t6, LL_Eng);
    // Left Engine
    btTransform t7;
    t7.setIdentity();
    t7.setOrigin(LL_Blt_Origin_Left_Eng);
    t7.getBasis().setEulerZYX(0, 0, SIMD_PI * Left_Eng_Direction / 180);  // Rotate -30° around Z axi
    LL_compound->addChildShape(t7, LL_Eng);
    // --- Rigid body ---
    btScalar body_mass = mass;
    btVector3 inertia(0, 0, 0);
    LL_compound->calculateLocalInertia(body_mass, inertia);
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(Px, Py, Pz));

    motionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(body_mass, motionState, LL_compound, inertia);
    LunLander = new btRigidBody(rbInfo);
    LunLander->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    LunLander->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    LunLander->setRestitution(Physic_LunarLander_Restitution);
    LunLander->setSleepingThresholds(0.5f, 0.5f);
    world->addRigidBody(LunLander);


    // body
    {
        LunrIdx = All_Rect_Shapes_Info.size();
        Rect_Shape_Info info;
        info.name = "LL Body";
        info.is_static = false;
        info.Blt_Body = LunLander;
        info.Blt_Offset = LL_Blt_Origin_body;
        Make_a_Rectangle(LL_Unit * LL_Body_Half_Size, LL_Unit * LL_Body_Half_Size, info, Color_LunarLander);
        All_Rect_Shapes_Info.push_back(info);
    }

    //  Arms
    {
        // Right
        Rect_Shape_Info info;
        info.name = "LL Right Arm";
        info.is_static = false;
        info.Blt_Body = LunLander;
        info.Blt_Offset = LL_Blt_Origin_Right_Arm;
        Make_a_Rectangle(LL_Unit * LL_Arm_Half_Width, LL_Unit * LL_Arm_Half_Height, info, Color_LunarLander);
        All_Rect_Shapes_Info.push_back(info);
        // Left
        info.name = "LL Left Arm";
        info.Blt_Offset = LL_Blt_Origin_Left_Arm;
        Make_a_Rectangle(LL_Unit * LL_Arm_Half_Width, LL_Unit * LL_Arm_Half_Height, info, Color_LunarLander);
        All_Rect_Shapes_Info.push_back(info);
    }

    // Legs
    {
        // Right
        Rect_Shape_Info info;
        info.name = "LL Right Leg";
        info.is_static = false;
        info.Blt_Body = LunLander;
        info.Blt_Offset = LL_Blt_Origin_Right_Leg;
        Make_a_Rectangle(LL_Unit * LL_Leg_Half_Width, LL_Unit * LL_Leg_half_Height, info, Color_LunarLander);
        All_Rect_Shapes_Info.push_back(info);
        // Left
        info.name = "LL Left Leg";
        info.Blt_Offset = LL_Blt_Origin_Left_Leg;
        Make_a_Rectangle(LL_Unit * LL_Leg_Half_Width, LL_Unit * LL_Leg_half_Height, info, Color_LunarLander);
        All_Rect_Shapes_Info.push_back(info);
    }

    //  Engines
    sf::Vector2f N1(-LL_Unit * LL_Eng_Half_Length * SCALE, LL_Unit * LL_Eng_Half_Length * SCALE);
    sf::Vector2f N2(LL_Unit * LL_Eng_Half_Length * SCALE, LL_Unit * LL_Eng_Half_Length * SCALE);
    sf::Vector2f N3(0, -LL_Unit * LL_Eng_Half_Length * SCALE);
    std::vector<sf::Vector2f> nodes = { N1 , N2, N3 };
    {
        // Right
        REngIdx = All_Cnvx_Shapes_Info.size();
        Cnvx_Shape_Info info;
        info.name = "LL Right Engine";
        info.is_static = false;
        info.Blt_Body = LunLander;
        info.Blt_Offset = LL_Blt_Origin_Right_Eng;
        Make_a_Cnvx(nodes, info, Color_LunarLander);
        All_Cnvx_Shapes_Info.push_back(info);
        // Left
        info.name = "LL Left Engine";
        LEngIdx = All_Cnvx_Shapes_Info.size();
        info.Blt_Offset = LL_Blt_Origin_Left_Eng;
        Make_a_Cnvx(nodes, info, Color_LunarLander);
        All_Cnvx_Shapes_Info.push_back(info);
    }
}

void LLEnv::Let_be_a_Platform_Flags()
{
    // Right
    Rect_Shape_Info Rinfo;
    Rinfo.name = "Right Flag Pole";
    Rinfo.is_static = true;
    Make_a_Rectangle(Flag_Pole_Half_Width, Flag_Pole_Half_Height, Rinfo, Color_Flag_Pole);
    Rinfo.SF_Shape.setPosition(toSFML({ Platform_center_X - Platform_flag_offcenter, Platform_Y + Flag_Pole_Half_Height, 0 }));
    All_Rect_Shapes_Info.push_back(Rinfo);
    // Left
    Rinfo.name = "Left Flag Pole";
    Rinfo.SF_Shape.setPosition(toSFML({ Platform_center_X + Platform_flag_offcenter, Platform_Y + Flag_Pole_Half_Height, 0 }));
    All_Rect_Shapes_Info.push_back(Rinfo);


    // Right
    Cnvx_Shape_Info Cinfo;
    Cinfo.name = "Right Flag Cloth";
    Cinfo.is_static = true;
    sf::Vector2f N1(0, -Flag_Cloth_Half_Height * SCALE);
    sf::Vector2f N2(0, +Flag_Cloth_Half_Height * SCALE);
    sf::Vector2f N3(-Flag_Cloth_Width * SCALE, 0);
    std::vector<sf::Vector2f> Rnodes = { N1 , N2, N3 };
    Make_a_Cnvx(Rnodes, Cinfo, Color_Flag_Cloth);
    Cinfo.SF_Shape.setPosition(toSFML({ Platform_center_X - Platform_flag_offcenter, Platform_Y + Flag_Pole_Half_Height, 0 }));
    All_Cnvx_Shapes_Info.push_back(Cinfo);
    // Left
    Cinfo.name = "Left Flag Cloth";
    std::vector<sf::Vector2f> Lnodes = { N1 , N2, -N3 };
    Make_a_Cnvx(Lnodes, Cinfo, Color_Flag_Cloth);
    Cinfo.SF_Shape.setPosition(toSFML({ Platform_center_X + Platform_flag_offcenter, Platform_Y + Flag_Pole_Half_Height, 0 }));
    All_Cnvx_Shapes_Info.push_back(Cinfo);
}

void LLEnv::Let_be_a_Engine_Vapor()
{
    REV = new EngineVapor(world);
    LEV = new EngineVapor(world);
}







sf::Vector2f LLEnv::True_Translation(const btVector3& localOffset, btTransform trans)
{
    btVector3 InThisWorld = trans * localOffset;   // rotates AND translates local offset
    return toSFML(InThisWorld);                    // convert to SFML pixels
}

void LLEnv::ApplyEngineThrust(const btVector3& localEnginePos, float localNozzleAngleDeg, float Force_Multiplier)
{
    LunLander->activate(true);
    // 1) Get world transform of the lander
    btTransform trans;
    LunLander->getMotionState()->getWorldTransform(trans);

    // 2) Engine world position (correctly rotated + translated)
    btVector3 worldEnginePos = trans * -localEnginePos;

    // 3) Build local nozzle direction:
    //    we assume "0 deg" nozzle points in local -Y (downwards in your lander local coordinates).
    //    Rotate that by localNozzleAngleDeg (positive = CCW) about Z (local).
    btScalar angleRad = localNozzleAngleDeg * SIMD_RADS_PER_DEG;
    btQuaternion qLocalRot(btVector3(0, 0, 1), angleRad); // rotate around local Z
    btVector3 localDown(0, -1, 0);                       // nozzle baseline points down in local coords
    btVector3 localExhaustDir = quatRotate(qLocalRot, localDown); // this is exhaust flow direction in local coords

    // 4) Transform exhaust direction to world space
    btVector3 worldExhaustDir = trans.getBasis() * localExhaustDir;
    worldExhaustDir.normalize();

    // IMPORTANT: force on the lander is OPPOSITE the exhaust flow direction
    btVector3 forceOnLander = -worldExhaustDir * (LL_Engine_thrust * Force_Multiplier);

    // 5) Apply force at engine world position
    //    applyForce(force, relPos) expects relPos = point of application relative to center of mass in world coords
    btVector3 relPos = worldEnginePos - LunLander->getCenterOfMassPosition();
    LunLander->applyForce(forceOnLander, relPos);

    //std::cout << 
    //    "Force: " << 
    //    forceOnLander.getX() << ", " << 
    //    forceOnLander.getY() << ", " <<
    //    forceOnLander.getZ() << "@:" <<
    //    relPos.getX() << ", " <<
    //    relPos.getY() << ", " <<
    //    relPos.getZ() << 
    //    std::endl;

    // --- DEBUG INFO (optional): print or draw arrow to verify sign/dir/torque ---
    // btVector3 torque = relPos.cross(forceOnLander);
    // std::cout << "relPos: " << relPos.x() << ","<< relPos.y() << "  force: " << forceOnLander.x() << "," << forceOnLander.y()
    //           << "  torque_z: " << torque.z() << std::endl;
}







