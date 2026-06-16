// main.cpp
// SFML + Box2D (v3.x) minimal example
// Requires: #include "box2d/box2d.h" from Box2D 3.x and SFML includes

#include "Shape.h"
#include "Physix.h"
#include "Utilities.h"
#include "Shader.h"
#include "Constants.h"
#include "Vapor.h"


#ifdef _WIN32
#include <windows.h>
extern "C" {
    __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

using namespace std;


#ifdef __TEST_CASE__
#ifdef __USE_BULLET__

#include <SFML/Graphics.hpp>
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <cmath>

sf::Vector2f bulletToSF(const btVector3& v, float scale = 100.f) {
    // Convert Bullet coordinates to SFML (flip Y)
    return sf::Vector2f(v.x() * scale, -v.y() * scale);
}

int main() {
    sf::RenderWindow window(sf::VideoMode({ 800, 600 }), "Compound Shape (Bullet + SFML)");
    window.setFramerateLimit(60);

    // --- Bullet setup ---
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    world->setGravity(btVector3(0, -9.8f, 0));

    // --- Compound shape ---
    btCompoundShape* compound = new btCompoundShape();

    // Green body
    btBoxShape* greenBody = new btBoxShape(btVector3(0.5f, 0.7f, 0)); // half extents
    btTransform tGreen; tGreen.setIdentity();
    tGreen.setOrigin(btVector3(0, 0, 0));
    compound->addChildShape(tGreen, greenBody);

    // Brown bar
    btBoxShape* brownBar = new btBoxShape(btVector3(0.8f, 0.1f, 0));
    btTransform tBar; tBar.setIdentity();
    tBar.setOrigin(btVector3(0, 0.8f, 0));
    compound->addChildShape(tBar, brownBar);

    // Blue legs
    btBoxShape* blueLeg = new btBoxShape(btVector3(0.1f, 0.15f, 0));
    btTransform tLegL; tLegL.setIdentity();
    tLegL.setOrigin(btVector3(-0.25f, -0.85f, 0));
    compound->addChildShape(tLegL, blueLeg);

    btTransform tLegR; tLegR.setIdentity();
    tLegR.setOrigin(btVector3(0.25f, -0.85f, 0));
    compound->addChildShape(tLegR, blueLeg);

    // Yellow triangles (approximate as boxes for physics)
    btBoxShape* yellowTri = new btBoxShape(btVector3(0.15f, 0.15f, 0));
    btTransform tTriL; tTriL.setIdentity();
    tTriL.setOrigin(btVector3(-0.95f, 0.8f, 0));
    compound->addChildShape(tTriL, yellowTri);

    btTransform tTriR; tTriR.setIdentity();
    tTriR.setOrigin(btVector3(0.95f, 0.8f, 0));
    compound->addChildShape(tTriR, yellowTri);

    // --- Rigid body ---
    btScalar mass = 1.0f;
    btVector3 inertia(0, 0, 0);
    compound->calculateLocalInertia(mass, inertia);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0, 2, 0));

    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, compound, inertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    world->addRigidBody(body);

    // --- Ground ---
    btBoxShape* groundShape = new btBoxShape(btVector3(10.0f, 0.5f, 0));
    btTransform tGround; tGround.setIdentity();
    tGround.setOrigin(btVector3(0, -2.0f, 0));
    btDefaultMotionState* gMotion = new btDefaultMotionState(tGround);
    btRigidBody::btRigidBodyConstructionInfo gInfo(0, gMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(gInfo);
    world->addRigidBody(groundBody);

    // --- SFML shapes ---
    const float SCALE = 100.f; // Bullet units -> pixels

    // Green body
    sf::RectangleShape green(sf::Vector2f(1.0f * 2 * SCALE * 0.5f, 1.4f * SCALE));
    green.setSize(sf::Vector2f(1.0f * SCALE, 1.4f * SCALE));
    green.setOrigin({ 0.5f * SCALE, 0.7f * SCALE });
    green.setFillColor(sf::Color(0, 170, 60));

    // Brown bar
    sf::RectangleShape brown(sf::Vector2f(1.6f * SCALE, 0.2f * SCALE));
    brown.setOrigin({ 0.8f * SCALE, 0.1f * SCALE });
    brown.setFillColor(sf::Color(166, 90, 60));

    // Blue legs
    sf::RectangleShape leg(sf::Vector2f(0.2f * SCALE, 0.3f * SCALE));
    leg.setOrigin({ 0.1f * SCALE, 0.15f * SCALE });
    leg.setFillColor(sf::Color(0, 0, 200));

    // Yellow triangles
    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, sf::Vector2f(-0.15f * SCALE, 0.15f * SCALE));
    tri.setPoint(1, sf::Vector2f(0.15f * SCALE, 0.15f * SCALE));
    tri.setPoint(2, sf::Vector2f(0, -0.15f * SCALE));
    tri.setFillColor(sf::Color(250, 230, 150));

    while (window.isOpen()) {
        while (auto eventOpt = window.pollEvent())  // pollEvent() returns std::optional<sf::Event>
        {
            if (!eventOpt) continue;
            sf::Event& event = *eventOpt;                 // dereference the optional

            if (event.is<sf::Event::Closed>())       // check if the event is "Closed"
                window.close();
        }

        // Step physics
        world->stepSimulation(1 / 60.f);

        // Get transform
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        btVector3 pos = trans.getOrigin();
        btQuaternion rot = trans.getRotation();

        float angleDeg = rot.getAngle() * rot.getAxis().z() * 180.f / 3.14159f;
        sf::Vector2f center = sf::Vector2f(400 + pos.x() * SCALE, 300 - pos.y() * SCALE);

        window.clear(sf::Color::White);

        // Draw green body
        green.setPosition(center);
        green.setRotation(sf::degrees(angleDeg));
        window.draw(green);

        // Draw brown bar
        brown.setPosition({ center.x, center.y });
        brown.move({ 0, -0.8f * SCALE });
        brown.setRotation(sf::degrees(angleDeg));
        window.draw(brown);

        // Draw legs
        leg.setPosition({ center.x - 0.25f * SCALE, center.y + 0.85f * SCALE });
        leg.setRotation(sf::degrees(angleDeg));
        window.draw(leg);

        leg.setPosition({ center.x + 0.25f * SCALE, center.y + 0.85f * SCALE });
        window.draw(leg);

        // Draw yellow triangles
        tri.setPosition({ center.x - 0.95f * SCALE, center.y - 0.8f * SCALE });
        tri.setRotation(sf::degrees(angleDeg));
        window.draw(tri);

        tri.setPosition({ center.x + 0.95f * SCALE, center.y - 0.8f * SCALE });
        window.draw(tri);

        window.display();
    }

    // Cleanup
    delete world;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
    return 0;
}


#else


int main()
{
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "Shapes Demo", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Load OpenGL with GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glfwSwapInterval(0); // disable V-Sync


    // Build shader program
    GLuint shaderProgram = createShaderProgram(vertexShaderSource, fragmentShaderSource);

    // Define shapes
    //std::vector<float> triVertices  = triangle_gen (0.5f, 0.6f,     0.6f, 0.6f,     0.6f, 0.5f);
    //std::vector<float> rectVertices = rectangle_gen(0.0f, 0.0f, 0.5f, 0.5f);
    //std::vector<unsigned int> rectIndices = Indices_gen(4);
    
    std::vector<float> cyclVertices = cycle_gen    (0.0f, 0.f, 0.5f, 360);
    std::vector<unsigned int> cyclIndices = Indices_gen(360);

    
    glUseProgram(shaderProgram);

    //Shape triangle (shaderProgram, triVertices);
    //Shape rectangle(shaderProgram, rectVertices, rectIndices);
    Shape cycle(shaderProgram, cyclVertices, cyclIndices);
    
    //triangle.color({ 1.0f, 0.0f, 0.0f });
    //rectangle.color({ 0.0f, 1.0f, 0.0f });
    cycle.color({ 0.0f, 0.0f, 1.0f });

    // Timing
    double startTime = glfwGetTime();

    // FPS counter
    int frames = 0;
    double lastTime = glfwGetTime();

    float rot(0.0f);
    float mov(0.0f);

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        double currentTime = glfwGetTime();
        float t = static_cast<float>(currentTime - startTime);

        mov = sin(rot);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        //triangle.move({ mov, 0 }); 
        //rectangle.rotate(rot);

        

        //triangle.draw();
        //rectangle.draw();
        cycle.draw();
        

        rot += 0.01;
        if (rot == 360.0f)
            rot =  0;

        glfwSwapBuffers(window);
        glfwPollEvents();


        // FPS counter (print every second)
        frames++;
        if (currentTime - lastTime >= 1.0)
        {
            std::cout << "FPS: " << frames << "\n";
            frames = 0;
            lastTime = currentTime;
        }

    }

    glfwTerminate();
    return 0;
}

#endif
#else 
#ifdef __USE_BULLET__ 

void applyEngineThrust(
    btRigidBody* lander,
    const btVector3& localEnginePos,
    float localNozzleAngleDeg,
    float Force_Multiplier)
{
    // 1) Get world transform of the lander
    btTransform trans;
    lander->getMotionState()->getWorldTransform(trans);

    // 2) Engine world position (correctly rotated + translated)
    btVector3 worldEnginePos = trans * localEnginePos;

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
    btVector3 relPos = worldEnginePos - lander->getCenterOfMassPosition();
    lander->applyForce(forceOnLander, relPos);

    // --- DEBUG INFO (optional): print or draw arrow to verify sign/dir/torque ---
    // btVector3 torque = relPos.cross(forceOnLander);
    // std::cout << "relPos: " << relPos.x() << ","<< relPos.y() << "  force: " << forceOnLander.x() << "," << forceOnLander.y()
    //           << "  torque_z: " << torque.z() << std::endl;
}

//void applyEngineThrust(btRigidBody* body, const btVector3& localEnginePos, btScalar Force_Multiplier, btScalar localAngleDeg)
//{
//    // Get world transform of the Lunar Lander
//    btTransform trans;
//    body->getMotionState()->getWorldTransform(trans);
//
//    // Convert engine position from local -> world
//    btVector3 worldPos = trans * localEnginePos;
//
//    // Compute local thrust direction (rotated by localAngle)
//    btScalar angleRad = SIMD_RADS_PER_DEG * localAngleDeg;
//    btVector3 localThrustDir(std::sin(angleRad), std::cos(angleRad), 0);  // diagonal outward thrust
//
//    // Transform to world space
//    btVector3 worldThrustDir = trans.getBasis() * localThrustDir;
//
//    // Apply force opposite to thrust direction
//    btVector3 force = worldThrustDir * (LL_Engine_thrust * Force_Multiplier);
//
//    // Apply the force at the world position of the engine
//    body->applyForce(force, worldPos - body->getCenterOfMassPosition());
//}


int main()
{
    std::random_device rd;                                // Non-deterministic random seed (if available)
    std::mt19937 gen(rd());                               // Mersenne Twister engine
    std::uniform_int_distribution<> intDist(1, 10);       // integers in [1, 10]
    std::uniform_real_distribution<> floatDist(0.0, 1.0); // floats in [0.0, 1.0]
    std::normal_distribution<> normDist(0.0, 1.0);        // mean = 0, stddev = 1


    // --- SFML Window ---
    sf::RenderWindow window(sf::VideoMode({ Window_Width, Window_Height }), "SFML + Bullet: Body + Arm");
    window.setFramerateLimit(Window_Max_FPS);


    // --- Bullet Setup ---
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btDbvtBroadphase* broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    world->setGravity(btVector3(Gravity_X, Gravity_Y, Gravity_Z));  // Y-down
    world->getSolverInfo().m_numIterations = 20;       // default 10
    world->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;


    // --- Ground ---
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* groundMotion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, Ground_Pos_Y, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundInfo(0, groundMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(groundInfo);
    groundBody->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    groundBody->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    groundBody->setRestitution(Ground_Restitution);
    world->addRigidBody(groundBody);

    sf::RectangleShape groundRect(sf::Vector2f(Window_Width, SCALE));
    groundRect.setOrigin({ Window_Width / 2, SCALE/2 });
    groundRect.setPosition({ Window_Width / 2, Window_Height - Ground_Pos_Y * SCALE/2 });
    groundRect.setFillColor(sf::Color::Green);


    // --- Body (box) ---
    btCollisionShape* bodyShape = new btBoxShape(btVector3(Body_Half_Width, Body_Half_Height, Body_Half_Depth)); // 1m × 1m x 1m
    btDefaultMotionState* bodyMotion = 
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(Body_Pos_X, Body_Pos_Y, Body_Pos_Z)));
    btScalar bodyMass = Body_mass;
    btVector3 bodyInertia(0, 0, 0);
    bodyShape->calculateLocalInertia(bodyMass, bodyInertia);
    btRigidBody::btRigidBodyConstructionInfo bodyInfo(bodyMass, bodyMotion, bodyShape, bodyInertia);
    btRigidBody* body = new btRigidBody(bodyInfo);
    body->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    body->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    body->setRestitution(Body_Restitution);
    world->addRigidBody(body);

    sf::RectangleShape bodyRect(sf::Vector2f(2 * Body_Half_Width * SCALE, 2 * Body_Half_Height * SCALE));
    bodyRect.setOrigin({ Body_Half_Width * SCALE, Body_Half_Height * SCALE });
    bodyRect.setFillColor(sf::Color::Red);


    //// --- Arm (thin rectangle) ---
    //btCollisionShape* armShape = new btBoxShape(btVector3(Arm_Half_Width, Arm_Half_Height, Arm_Half_Depth)); // 3m × 0.2m * 1m
    //btDefaultMotionState* armMotion = 
    //    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(Arm_Pos_X, Arm_Pos_Y, Arm_Pos_Z)));
    //btScalar armMass = Arm_mass;
    //btVector3 armInertia(0, 0, 0);
    //armShape->calculateLocalInertia(armMass, armInertia);
    //btRigidBody::btRigidBodyConstructionInfo armInfo(armMass, armMotion, armShape, armInertia);
    //btRigidBody* arm = new btRigidBody(armInfo);
    //arm->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    //arm->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    //arm->setRestitution(Arm_Restitution);
    //world->addRigidBody(arm);
    //
    //sf::RectangleShape armRect(sf::Vector2f(2 * Arm_Half_Width * SCALE, 2 * Arm_Half_Height * SCALE));
    //armRect.setOrigin({ Arm_Half_Width * SCALE, Arm_Half_Height * SCALE });
    //armRect.setFillColor(sf::Color::Blue);
    //
    //
    //// --- Circle (ball) ---
    //btCollisionShape* circleShape = new btSphereShape(Circle_Radius);  // keep units consistent with SCALE
    //btDefaultMotionState* circleMotionState = 
    //    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(Circle_Pos_X, Circle_Pos_Y, Circle_Pos_Z)));
    //btScalar circleMass = Circle_mass;
    //btVector3 circleInertia(0, 0, 0);
    //circleShape->calculateLocalInertia(circleMass, circleInertia);
    //btRigidBody::btRigidBodyConstructionInfo circleRigidBodyCI(circleMass, circleMotionState, circleShape, circleInertia);
    //btRigidBody* Circ = new btRigidBody(circleRigidBodyCI);
    //// lock Z movement and rotation -> pure 2D
	//Circ->setRestitution(Circle_Restitution);
    //Circ->setLinearFactor(btVector3(1, 1, 0));
    //Circ->setAngularFactor(btVector3(0, 0, 1));
    //world->addRigidBody(Circ);
    //
    //// --- SFML visual ---
    //sf::CircleShape circleVisual(Circle_Radius * SCALE);
    //circleVisual.setOrigin({ Circle_Radius * SCALE, Circle_Radius * SCALE });
    //circleVisual.setFillColor(sf::Color::Green);
    //
    //
    //// --- Mountain Mesh ---
    //// Create triangle mesh
    //btTriangleMesh* mountainMesh = new btTriangleMesh();
    //bool before(true);
    //bool after(false);
    //std::vector<sf::ConvexShape> mountains;
    //float x_base(0);
    //float y_base(Ground_Pos_Y + Mount_Ave_Height + Mount_Max_DeltaY * normDist(gen));
    //float x_next(0);
    //float y_next(0);
    //float x_prev(0);
    //float y_prev(0);
    //while (x_next <= Mount_length)
    //{
    //    sf::ConvexShape mount1;
    //    sf::ConvexShape mount2;
    //    mount1.setPointCount(3);
    //    mount2.setPointCount(3);
    //
    //    float random_Height = normDist(gen);
    //    float random_length = floatDist(gen);
    //
    //    
    //    if (before || after)
    //    {
    //        float diff = std::abs(Platcorm_center_X - x_next);
    //        x_next = x_base + random_length * Mount_Max_DeltaX;
    //        y_next = std::min(Platform_Y + (diff * Mount_max_slope), y_base + random_Height * Mount_Max_DeltaY);
    //    }
    //    else
    //    {
    //        x_next = Platform_X + Platform_Length;
    //        after  = true;
    //    }
    //
    //    if ((x_next >= Platform_X) && before)
    //    {
    //        x_next = Platform_X;
    //        y_next = Platform_Y;
    //        before = false;
    //    }
    //
    //    mountainMesh->addTriangle(
    //        btVector3(x_base, Ground_Pos_Y, 0),
    //        btVector3(x_base, y_base,       0),
    //        btVector3(x_next, y_next,       0));
    //    mountainMesh->addTriangle(
    //        btVector3(x_base, Ground_Pos_Y, 0),
    //        btVector3(x_next, y_next, 0),
    //        btVector3(x_next, Ground_Pos_Y, 0));
    //
    //
    //    mount1.setPoint(0, toSFML({ x_base,    Ground_Pos_Y,     0 }));
    //    mount1.setPoint(1, toSFML({ x_base,    y_base,           0 }));
    //    mount1.setPoint(2, toSFML({ x_next,    y_next,           0 }));
    //    mount2.setPoint(0, toSFML({ x_base,    Ground_Pos_Y,     0 }));
    //    mount2.setPoint(1, toSFML({ x_next,    y_next,           0 }));
    //    mount2.setPoint(2, toSFML({ x_next,    Ground_Pos_Y,     0 }));
    //
    //    x_prev = x_base;
    //    y_prev = y_base;
    //    x_base = x_next;
    //    y_base = y_next;
    //
    //
    //
    //    mount1.setFillColor(sf::Color(139, 69, 19)); // brownish mountain
    //    mount2.setFillColor(sf::Color(139, 69, 19)); // brownish mountain
    //    mountains.push_back(mount1);
    //    mountains.push_back(mount2);
    //}
    //
    //// Create static concave shape
    //btBvhTriangleMeshShape* mountainShape = new btBvhTriangleMeshShape(mountainMesh, true);
    //// No mass -> static
    //btDefaultMotionState* mountainMotionState = new btDefaultMotionState(
    //    btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))
    //);
    //btRigidBody::btRigidBodyConstructionInfo mountainCI(0.0f, mountainMotionState, mountainShape, btVector3(0, 0, 0));
    //btRigidBody* mountainBody = new btRigidBody(mountainCI);
    //mountainBody->setRestitution(Mount_Restitution);
    //world->addRigidBody(mountainBody);

    



    // --- Create a compound shape ---
    btCompoundShape* compound = new btCompoundShape();

    // Green body
    // bullet
    btBoxShape* LLbody = new btBoxShape(btVector3(LL_Unit* LL_Body_Half_Size, LL_Unit* LL_Body_Half_Size, LL_Depth));
    btTransform t1;
    t1.setIdentity();
    t1.setOrigin(LL_Blt_Origin_body);
    compound->addChildShape(t1, LLbody);
    // SFML
    sf::RectangleShape LL_body_shape(sf::Vector2f(2 * LL_Unit * LL_Body_Half_Size * SCALE, 2 * LL_Unit * LL_Body_Half_Size * SCALE));
    LL_body_shape.setSize({ 2 * LL_Unit * LL_Body_Half_Size * SCALE, 2 * LL_Unit * LL_Body_Half_Size * SCALE });
    //LL_body_shape.setOrigin(sf::Vector2f(LL_SF_Origin_body) + sf::Vector2f(LL_SF_offset_body));
    LL_body_shape.setOrigin(sf::Vector2f(LL_SF_Origin_body));
    LL_body_shape.setFillColor(sf::Color(0, 170, 60));


    // Arm
    // Right Arm
    // Bullet
    btBoxShape* LL_Arm = new btBoxShape(btVector3(LL_Unit* LL_Arm_Half_Width, LL_Unit* LL_Arm_Half_Height, LL_Depth));
    btTransform t2;
    t2.setIdentity();
    t2.setOrigin(LL_Blt_Origin_Right_Arm);
    compound->addChildShape(t2, LL_Arm);
    // SFML
    sf::RectangleShape LL_Right_Arm_shape(sf::Vector2f(2 * LL_Unit * LL_Arm_Half_Width * SCALE, 2 * LL_Unit * LL_Arm_Half_Height * SCALE));
    //LL_Right_Arm_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Arm) + sf::Vector2f(LL_SF_offset_Right_Arm));
    LL_Right_Arm_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Arm));
    LL_Right_Arm_shape.setFillColor(sf::Color(166, 90, 60));
    // Left Arm
    // Bullet
    btTransform t3;
    t3.setIdentity();
    t3.setOrigin(LL_Blt_Origin_Left_Arm);
    compound->addChildShape(t3, LL_Arm);
    // SFML
    sf::RectangleShape LL_Left_Arm_shape(sf::Vector2f(2 * LL_Unit * LL_Arm_Half_Width * SCALE, 2 * LL_Unit * LL_Arm_Half_Height * SCALE));
    //LL_Left_Arm_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Arm) + sf::Vector2f(LL_SF_offset_Left_Arm));
    LL_Left_Arm_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Arm));
    LL_Left_Arm_shape.setFillColor(sf::Color(166, 90, 60));


    // legs
    // Right Leg
    // Bullet
    btBoxShape* LL_leg = new btBoxShape(btVector3(LL_Unit * LL_Leg_Half_Width, LL_Unit * LL_Leg_half_Height, LL_Depth));
    btTransform t4;
    t4.setIdentity();
    t4.setOrigin(LL_Blt_Origin_Right_Leg);
    compound->addChildShape(t4, LL_leg);
    // SFML
    sf::RectangleShape LL_Right_Leg_shape(sf::Vector2f(2 * LL_Unit * LL_Leg_Half_Width * SCALE, 2 * LL_Unit * LL_Leg_half_Height * SCALE));
    //LL_Right_Leg_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Leg) + sf::Vector2f(LL_SF_offset_Right_Leg));
    LL_Right_Leg_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Leg));
    LL_Right_Leg_shape.setFillColor(sf::Color(166, 90, 60));
    // Left Leg
    // Bullet
    btTransform t5;
    t5.setIdentity();
    t5.setOrigin(LL_Blt_Origin_Left_Leg);
    compound->addChildShape(t5, LL_leg);
    // SFML
    sf::RectangleShape LL_Left_Leg_shape(sf::Vector2f(2 * LL_Unit * LL_Leg_Half_Width * SCALE, 2 * LL_Unit * LL_Leg_half_Height * SCALE));
    //LL_Left_Leg_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Leg) + sf::Vector2f(LL_SF_offset_Left_Leg));
    LL_Left_Leg_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Leg));
    LL_Left_Leg_shape.setFillColor(sf::Color(166, 90, 60));


    // Engine
    // Right Engine
    // bullet
    btBoxShape* LL_Eng = new btBoxShape(btVector3(LL_Unit * LL_Eng_Half_Length, LL_Unit * LL_Eng_Half_Length, LL_Depth));
    btTransform t6;
    t6.setIdentity();
    t6.setOrigin(LL_Blt_Origin_Right_Eng);
    t6.getBasis().setEulerZYX(0, 0, -SIMD_PI / 6);  // Rotate -30° around Z axis
    compound->addChildShape(t6, LL_Eng);
    // SFML
    // Yellow triangles
    sf::ConvexShape LL_Right_Eng_shape;
    LL_Right_Eng_shape.setPointCount(3);
    LL_Right_Eng_shape.setPoint(0, sf::Vector2f(-LL_Unit*LL_Eng_Half_Length * SCALE, LL_Unit*LL_Eng_Half_Length* SCALE));
    LL_Right_Eng_shape.setPoint(1, sf::Vector2f(LL_Unit*LL_Eng_Half_Length* SCALE, LL_Unit*LL_Eng_Half_Length* SCALE));
    LL_Right_Eng_shape.setPoint(2, sf::Vector2f(0, -LL_Unit * LL_Eng_Half_Length * SCALE));
    //LL_Right_Eng_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Eng) + sf::Vector2f(LL_SF_offset_Right_Eng));
    LL_Right_Eng_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Right_Eng));
    LL_Right_Eng_shape.setFillColor(sf::Color(250, 230, 150));
    // Left Engine
    // bullet
    btTransform t7;
    t7.setIdentity();
    t7.setOrigin(LL_Blt_Origin_Left_Eng);
    t7.getBasis().setEulerZYX(0, 0, +SIMD_PI / 6);  // Rotate -30° around Z axi
    compound->addChildShape(t7, LL_Eng);
    // SFML
    // Yellow triangles
    sf::ConvexShape LL_Left_Eng_shape;
    LL_Left_Eng_shape.setPointCount(3);
    LL_Left_Eng_shape.setPoint(0, sf::Vector2f(-LL_Unit*LL_Eng_Half_Length * SCALE, LL_Unit*LL_Eng_Half_Length* SCALE));
    LL_Left_Eng_shape.setPoint(1, sf::Vector2f(LL_Unit*LL_Eng_Half_Length* SCALE, LL_Unit*LL_Eng_Half_Length* SCALE));
    LL_Left_Eng_shape.setPoint(2, sf::Vector2f(0, -LL_Unit*LL_Eng_Half_Length * SCALE));
    //LL_Left_Eng_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Eng) + sf::Vector2f(LL_SF_offset_Left_Eng));
    LL_Left_Eng_shape.setOrigin(sf::Vector2f(LL_SF_Origin_Left_Eng));
    LL_Left_Eng_shape.setFillColor(sf::Color(250, 230, 150));


    // --- Rigid body ---
    btScalar mass = LL_mass;
    btVector3 inertia(0, 0, 0);
    compound->calculateLocalInertia(mass, inertia);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(LL_Pos_X, LL_Pos_Y, 0));

    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, compound, inertia);
    btRigidBody* LunLander = new btRigidBody(rbInfo);
    LunLander->setLinearFactor(btVector3(1, 1, 0));   // allow X, Y movement only
    LunLander->setAngularFactor(btVector3(0, 0, 1));  // allow rotation only around Z axis
    LunLander->setRestitution(LL_Restitution);
    world->addRigidBody(LunLander);









    EngineVapor REV(world);
    EngineVapor LEV(world);



    
        


    //// Flags
    //sf::RectangleShape F1_pole(sf::Vector2f(2 * Flag_Pole_Half_Width * SCALE, 2 * Flag_Pole_Half_Height * SCALE));
    //F1_pole.setOrigin({ 0, 0 });
    //F1_pole.setPosition({ 0, 0 });
    //F1_pole.setFillColor(sf::Color::Blue);
    //F1_pole.setPosition(toSFML({ Platcorm_center_X - Platform_flag_offcenter, Platform_Y + 2*Flag_Pole_Half_Height, 0}));
    //
    //sf::RectangleShape F2_pole(sf::Vector2f(2 * Flag_Pole_Half_Width * SCALE, 2 * Flag_Pole_Half_Height * SCALE));
    //F2_pole.setOrigin({ 0, 0 });
    //F2_pole.setPosition({ 0, 0 });
    //F2_pole.setFillColor(sf::Color::Blue);
    //F2_pole.setPosition(toSFML({ Platcorm_center_X + Platform_flag_offcenter, Platform_Y + 2*Flag_Pole_Half_Height, 0 }));
    //
    //
    //sf::ConvexShape F1_cloth;
    //F1_cloth.setPointCount(3);
    //F1_cloth.setPoint(0, sf::Vector2f(0, -Flag_Cloth_Half_Height * SCALE));
    //F1_cloth.setPoint(1, sf::Vector2f(0, +Flag_Cloth_Half_Height * SCALE));
    //F1_cloth.setPoint(2, sf::Vector2f(-Flag_Cloth_Width* SCALE, 0));
    //F1_cloth.setOrigin({ 0, Flag_Cloth_Half_Height });
    //F1_cloth.setFillColor(sf::Color::Green);
    //F1_cloth.setPosition(toSFML({ Platcorm_center_X - Platform_flag_offcenter, Platform_Y+2* Flag_Pole_Half_Height, 0 }));
    //
    //
    //sf::ConvexShape F2_cloth;
    //F2_cloth.setPointCount(3);
    //F2_cloth.setPoint(0, sf::Vector2f(0, -Flag_Cloth_Half_Height * SCALE));
    //F2_cloth.setPoint(1, sf::Vector2f(0, +Flag_Cloth_Half_Height * SCALE));
    //F2_cloth.setPoint(2, sf::Vector2f(+Flag_Cloth_Width* SCALE, 0));
    //F2_cloth.setOrigin({ 0, Flag_Cloth_Half_Height });
    //F2_cloth.setFillColor(sf::Color::Green);
    //F2_cloth.setPosition(toSFML({ Platcorm_center_X + Platform_flag_offcenter, Platform_Y + 2 * Flag_Pole_Half_Height, 0 }));




    // --- Hinge joint between body and arm ---
    //btVector3 pivotBody(0, 0.5f, 0);
    //btVector3 pivotArm(0, -0.1f, 0);
    //btVector3 axis(0, 0, 1);        // rotation axis (Z in 2D)
    //btHingeConstraint* hinge = new btHingeConstraint(*body, *arm, pivotBody, pivotArm, axis, axis);
    //world->addConstraint(hinge, true);

    // --- Mouse drag variables ---
    bool dragging = false;
    btRigidBody* pickedBody = nullptr;
    btVector3 pickOffset(0, 0, 0);
    bool Force_applied_left(false);
    bool Force_applied_right(false);
    bool shift_pressed(false);
    // --- Main Loop ---
    while (window.isOpen())
    {
        while (auto eventOpt = window.pollEvent())  // pollEvent() returns std::optional<sf::Event>
        {
            if (!eventOpt) continue;
            sf::Event& event = *eventOpt;                 // dereference the optional

            if (event.is<sf::Event::Closed>())       // check if the event is "Closed"
                window.close();

            if (event.is<sf::Event::MouseButtonPressed>())
            {
                auto mouseEvent = event.getIf<sf::Event::MouseButtonPressed>();
                if (mouseEvent->button == sf::Mouse::Button::Left)
                {
                    sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
                    float mx = mousePixel.x / SCALE;
                    float my = (Window_Height - mousePixel.y) / SCALE;

                    btVector3 from(mx, my, 1);
                    btVector3 to(mx, my, -1);
                    btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
                    world->rayTest(from, to, rayCallback);

                    if (rayCallback.hasHit())
                    {
                        pickedBody = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
                        if (pickedBody && !pickedBody->isStaticObject())
                        {
                            dragging = true;
                            pickOffset = pickedBody->getCenterOfMassTransform().getOrigin() - rayCallback.m_hitPointWorld;
                        }
                    }
                }
            }

            if (event.is<sf::Event::MouseButtonReleased>())
            {
                auto mouseEvent = event.getIf<sf::Event::MouseButtonReleased>();
                if (mouseEvent->button == sf::Mouse::Button::Left)
                {
                    dragging = false;
                    pickedBody = nullptr;
                }
            }
        }

        // --- Input Handling ---
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
        {
            body->activate(true);
            body->applyCentralForce(btVector3(-50, 0, 0));
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
        {
            body->activate(true);
            body->applyCentralForce(btVector3(50, 0, 0));
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
        {
            body->activate(true);
            body->applyCentralForce(btVector3(0, 50, 0));
        }

        shift_pressed = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space);
        Force_applied_left  = false;
        Force_applied_right = false;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
        {
            // Apply thrust on the left engine
            LunLander->activate(true);
            Force_applied_left = true;
            applyEngineThrust(LunLander, LL_Blt_Origin_Right_Eng, -LL_Engine_offcenter_rot, 1 + shift_pressed);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
        {
            // Apply thrust on the right engine
            LunLander->activate(true);
            Force_applied_right = true;
            applyEngineThrust(LunLander, LL_Blt_Origin_Left_Eng,  +LL_Engine_offcenter_rot, 1 + shift_pressed);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
        {
            // Apply both engines for upward thrust
            LunLander->activate(true);
            if (!Force_applied_left)  applyEngineThrust(LunLander, LL_Blt_Origin_Right_Eng,-LL_Engine_offcenter_rot, 1.0 + shift_pressed);
            if (!Force_applied_right) applyEngineThrust(LunLander, LL_Blt_Origin_Left_Eng, +LL_Engine_offcenter_rot, 1.0 + shift_pressed);
            Force_applied_left  = true;
            Force_applied_right = true;
        }

        // --- Step simulation ---
        world->stepSimulation(1.0f / Window_Max_FPS, 32);

        // --- Apply dragging force ---
        if (dragging && pickedBody)
        {
            sf::Vector2i mouse = sf::Mouse::getPosition(window);
            btVector3 target((float)mouse.x / SCALE, (Window_Height - mouse.y) / SCALE, 0);
            btVector3 current = pickedBody->getCenterOfMassTransform().getOrigin();
            btVector3 force = (target - current - pickOffset) * 20.0f;
            pickedBody->applyCentralForce(force);
            pickedBody->activate(true);
        }

        // --- Draw ---
        window.clear(sf::Color(20, 20, 30));  // dark gray-blue

        // Body transform
        {
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            btVector3 pos = trans.getOrigin();
            sf::Vector2f drawPos = toSFML(pos);
            btQuaternion rot = trans.getRotation();
            btScalar angle = rot.getAngle() * (180.0f / btScalar(M_PI));
            if (rot.getAxis().getZ() < 0) angle = -angle;
            bodyRect.setPosition(drawPos);
            bodyRect.setRotation(sf::degrees (- angle));
            window.draw(bodyRect);
        }

        //// Arm transform
        //{
        //    btTransform trans;
        //    arm->getMotionState()->getWorldTransform(trans);
        //    btVector3 pos = trans.getOrigin();
        //    sf::Vector2f drawPos = toSFML(pos);
        //    btQuaternion rot = trans.getRotation();
        //    btScalar angle = rot.getAngle() * (180.0f / btScalar(M_PI));
        //    if (rot.getAxis().getZ() < 0) angle = -angle;
        //    armRect.setPosition(drawPos);
        //    armRect.setRotation(sf::degrees(-angle));
        //    window.draw(armRect);
        //}
        //
        //// Circ transform
        //{
        //    btTransform trans;
        //    Circ->getMotionState()->getWorldTransform(trans);
        //    btVector3 pos = trans.getOrigin();
        //    sf::Vector2f drawPos = toSFML(pos);
        //    circleVisual.setPosition(drawPos);
        //    window.draw(circleVisual);
        //}
        

        // LunLander
        {
            // Get lander transform & position/orientation
            btTransform trans;
            LunLander->getMotionState()->getWorldTransform(trans);
            btVector3 bodyPos = trans.getOrigin();
            btQuaternion rot = trans.getRotation();


            // Convert Bullet quaternion to a single 2D rotation angle (degrees)
            // getAngle() returns magnitude, need sign from axis.z
            btScalar angleRad = rot.getAngle();
            if (rot.getAxis().getZ() < 0) angleRad = -angleRad;
            float angleDeg = angleRad * (180.0f / (float)M_PI);
            

            // --- Body (center) ---
            {
                sf::Vector2f drawBodyPos = toSFML(bodyPos);
                LL_body_shape.setPosition(drawBodyPos);
                LL_body_shape.setRotation(sf::degrees(-angleDeg)); // SFML uses degrees CCW positive
            }

            // Helper lambda: compute world position of a child given its local Bullet offset macro
            auto childWorldSF = [&](const btVector3& localOffset) -> sf::Vector2f
            {
                btVector3 world = trans * localOffset;   // rotates AND translates local offset
                return toSFML(world);                    // convert to SFML pixels
            };

            // --- Right Arm ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Right_Arm);
                LL_Right_Arm_shape.setPosition(p);
                LL_Right_Arm_shape.setRotation(sf::degrees(-angleDeg));
            }

            // --- Left Arm ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Left_Arm);
                LL_Left_Arm_shape.setPosition(p);
                LL_Left_Arm_shape.setRotation(sf::degrees(-angleDeg));
            }

            // --- Right Leg ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Right_Leg);
                LL_Right_Leg_shape.setPosition(p);
                LL_Right_Leg_shape.setRotation(sf::degrees(-angleDeg));
            }

            // --- Left Leg ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Left_Leg);
                LL_Left_Leg_shape.setPosition(p);
                LL_Left_Leg_shape.setRotation(sf::degrees(-angleDeg));
            }

            // --- Right Engine ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Right_Eng);
                LL_Right_Eng_shape.setPosition(p);
                LL_Right_Eng_shape.setRotation(sf::degrees(-angleDeg + LL_Engine_offcenter_rot));
            }

            // --- Left Engine ---
            {
                sf::Vector2f p = childWorldSF(LL_Blt_Origin_Left_Eng);
                LL_Left_Eng_shape.setPosition(p);
                LL_Left_Eng_shape.setRotation(sf::degrees(-angleDeg - LL_Engine_offcenter_rot));
            }


            window.draw(LL_body_shape     );
            window.draw(LL_Right_Arm_shape);
            window.draw(LL_Left_Arm_shape );
            window.draw(LL_Right_Leg_shape);
            window.draw(LL_Left_Leg_shape );
            window.draw(LL_Right_Eng_shape);
            window.draw(LL_Left_Eng_shape );
        }


        // Rigt Engine Vapor
        {
            btTransform trans;
            LunLander->getMotionState()->getWorldTransform(trans);
            btVector3 worldEnginePos = trans * -btVector3(LL_Blt_Origin_Right_Eng);


            // Local direction of exhaust (in lander’s local space)
            btVector3 worldDir = trans.getBasis() * btVector3(+std::sin(LL_Engine_offcenter_rot * 180.0f / (float)M_PI), +std::cos(LL_Engine_offcenter_rot * 180.0f / (float)M_PI), 0);
            worldDir.normalize();
            worldDir = Vapor_ejection_speed_base * worldDir;

            // fixed location for testing
            REV.setEmitter(worldEnginePos);
            REV.setDirection(worldDir);
            
            REV.emit(Vapor_cnt * Force_applied_right, 1 + shift_pressed);
            REV.update(1.0f / Window_Max_FPS);
            REV.draw(window);
        }


        // Left Engine Vapor
        {
            btTransform trans;
            LunLander->getMotionState()->getWorldTransform(trans);
            btVector3 worldEnginePos = trans * -btVector3(LL_Blt_Origin_Left_Eng);


            // Local direction of exhaust (in lander’s local space)
            btVector3 worldDir = trans.getBasis() * btVector3(-std::sin(LL_Engine_offcenter_rot * 180.0f / (float)M_PI),+std::cos(LL_Engine_offcenter_rot * 180.0f / (float)M_PI), 0);
            worldDir.normalize();
            worldDir = Vapor_ejection_speed_base * worldDir;

            // fixed location for testing
            LEV.setEmitter(worldEnginePos);
            LEV.setDirection(worldDir);
            
            LEV.emit(Vapor_cnt * Force_applied_left, 1 + shift_pressed);
            LEV.update(1.0f / Window_Max_FPS);
            LEV.draw(window);
        }


        //for (auto& mount : mountains)
        //{
        //    window.draw(mount);
        //}
        


        //window.draw(F1_pole);
        //window.draw(F2_pole);
        //window.draw(F1_cloth);
        //window.draw(F2_cloth);



        window.draw(groundRect);
        window.display();
    }

    // --- Cleanup ---
    delete world;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
    return 0;
}


#else

int main()
{
    int GID;
    std::vector<int> BID;

    b2Vec2 gravity = { 0.0f, 9.8f };                       // gravity downward in m/s^2
    b2Vec2 windForce = { 0.0f, 0.0f };
    b2Vec2 tot_force = combine(gravity, windForce);

    float ground_friction(0.2f);
    float ground_restitution(0.5f);
    float ground_half_width(50.0f);
    float ground_half_height(1.0f);

    b2Vec2 Ground_pos = { 0.0f, 10.0f };

    float  Box1_friction(0.2f);
    float  Box1_restitution(0.5f);
    float  Box1_half_width(1.0f);
    float  Box1_half_height(1.0f);
    float  Box1_Density(5.0f);
    b2Vec2 Box1_Shape_Spos = {-2.0f, 0.0f };

    float  Box2_friction(0.2f);
    float  Box2_restitution(0.5f);
    float  Box2_half_width(0.25f);
    float  Box2_half_height(0.5f);
    float  Box2_Density(5.0f);
    b2Vec2 Box2_Shape_Spos = { -1.125f , -0.75f };

    float  Circle_friction(0.2f);
    float  Circle_restitution(0.0f);
    float  Circle_half_width(1.0f);
    float  Circle_half_height(1.0f);
    float  Circle_radios(1.0f);
    float  Circle_Density(5.0f);
    b2Vec2 Cir_Shape_Spos = { 2.0f, -4.0f };
    b2Vec2 Circle_offset = Cir_Shape_Spos - b2Vec2{Circle_radios, Circle_radios};


    float damping_factor(0.9f);



    physics phy(tot_force);
    if (!phy.is_the_world_available())
        return -1;


    // --- Create ground (static body) ---
    b2BodyDef groundDef = b2DefaultBodyDef();
    groundDef.type = b2_staticBody;
    groundDef.position = Ground_pos;
    //              and
    // Create a big horizontal box shape for the ground
    b2Polygon groundPoly = b2MakeBox(ground_half_width, ground_half_height);
    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    groundShapeDef.material.friction = ground_friction;
    groundShapeDef.material.restitution = ground_restitution;
    //              adding
    GID = phy.add_body_and_shape(groundDef, groundPoly, groundShapeDef);



    // --- Create dynamic body (a falling box) ---
    b2BodyDef Box1Def = b2DefaultBodyDef();
    Box1Def.type = b2_dynamicBody;
    Box1Def.position = Box1_Shape_Spos; // start at origin (meters)
    Box1Def.motionLocks = { false, false, true };
    //Box1Def.linearDamping = 2.0f;
    //              and
    // make a 2x2 meter box (half extents 1.0)
    b2Polygon Box1_poly = b2MakeBox(Box1_half_width, Box1_half_height);
    b2ShapeDef Box1_ShapeDef = b2DefaultShapeDef();
    Box1_ShapeDef.density = Box1_Density;
    Box1_ShapeDef.material.friction = Box1_friction;
    Box1_ShapeDef.material.restitution = Box1_restitution;
    //              adding
    BID.push_back(phy.add_body_and_shape(Box1Def, Box1_poly, Box1_ShapeDef));



    // --- Create dynamic body (a falling box) ---
    b2BodyDef circDef = b2DefaultBodyDef();
    circDef.type = b2_dynamicBody;
    circDef.position = Cir_Shape_Spos; // start at origin (meters)
    circDef.motionLocks = { false, false, false };
    //circDef.linearDamping = 1.0f;
    //              and
    // make a 2x2 meter box (half extents 1.0)
    b2Circle  dynamicCir = { Cir_Shape_Spos, Circle_radios};
    b2ShapeDef shapeDefCir = b2DefaultShapeDef();
    shapeDefCir.density = Circle_Density;
    shapeDefCir.material.friction = Circle_friction;
    shapeDefCir.material.restitution = Circle_restitution;
    //              adding
    BID.push_back(phy.add_body_and_shape(circDef, dynamicCir, shapeDefCir));
    



    // --- Create dynamic body (a falling box) ---
    b2BodyDef Box2Def = b2DefaultBodyDef();
    Box2Def.type = b2_dynamicBody;
    Box2Def.position = Box2_Shape_Spos; // start at origin (meters)
    Box2Def.motionLocks = { false, false, false };
    //Box2Def.linearDamping = 2.0f;
    //              and
    // make a 2x2 meter box (half extents 1.0)
    b2Polygon Box2_poly = b2MakeBox(Box2_half_width, Box2_half_height);
    b2ShapeDef Box2_ShapeDef = b2DefaultShapeDef();
    Box2_ShapeDef.density = Box2_Density;
    Box2_ShapeDef.material.friction = Box2_friction;
    Box2_ShapeDef.material.restitution = Box2_restitution;
    //              adding
    BID.push_back(phy.add_body_and_shape(Box2Def, Box2_poly, Box2_ShapeDef));





    // Define a revolute joint
    b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

    b2BodyId bodyA = phy.get_body_ID(BID[0]); // base (Box1)
    b2BodyId bodyB = phy.get_body_ID(BID[2]); // arm   (Box2)

    // Decide the world-space anchor where the hinge should be.
    // Example: hinge at Box1 right edge (base position.x + half width)
    b2Vec2 basePos = b2Body_GetPosition(bodyA);
    b2Vec2 worldAnchor = { basePos.x + Box1_half_width, basePos.y }; // hinge on right edge of Box1

    // Assign bodies
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;

    // Convert the same world anchor into each body's local frame (crucial!)
    jointDef.base.localFrameA.p = b2Body_GetLocalPoint(bodyA, worldAnchor);
    jointDef.base.localFrameB.p = b2Body_GetLocalPoint(bodyB, worldAnchor);
    
    // Zero rotations of local frames (safer if your API expects this)
    jointDef.base.localFrameA.q = b2Rot{ 1.0f, 0.0f }; // identity rotation (cos=1,sin=0)
    jointDef.base.localFrameB.q = b2Rot{ 1.0f, 0.0f };

    // Let the arm dangle freely (no tight motor). Enable limits only if you want range.
    jointDef.enableLimit = false;
    jointDef.enableMotor = false;
    jointDef.maxMotorTorque = 0.0f;

    // Create the joint
    b2JointId jointId = b2CreateRevoluteJoint(phy.get_world_ID(), &jointDef);
    
    
    





    // --- SFML window ---
    sf::RenderWindow window(sf::VideoMode({ Window_Width, Window_Height }), "SFML + Box2D v3 example");
    window.setFramerateLimit(Window_Max_FPS);


    // ground visual (very wide)
    sf::RectangleShape groundRect(sf::Vector2f(100.0f * PIXELS_PER_METER, 2.0f * PIXELS_PER_METER));
    groundRect.setFillColor(sf::Color(100, 100, 100));
    groundRect.setOrigin({ groundRect.getSize().x / 2.f, groundRect.getSize().y / 2.f });



    // rectangle used to draw the box (size in pixels)
    sf::RectangleShape rect1(sf::Vector2f(2 * Box1_half_width * PIXELS_PER_METER, 2 * Box1_half_height * PIXELS_PER_METER));
    rect1.setFillColor(sf::Color::Red);
    rect1.setOrigin({ rect1.getSize().x / 2.f, rect1.getSize().y / 2.f });
            

    // circle used to draw the circle (size in pixels)
    sf::CircleShape circ(Circle_radios * PIXELS_PER_METER, 64);
    circ.setFillColor(sf::Color::Green);
    circ.setOrigin({ 0.0f, 0.0f });

    // rectangle used to draw the box (size in pixels)
    sf::RectangleShape rect2(sf::Vector2f(2 * Box2_half_width * PIXELS_PER_METER, 2 * Box2_half_height * PIXELS_PER_METER));
    rect2.setFillColor(sf::Color::Blue);
    rect2.setOrigin({ rect2.getSize().x / 2.f, rect2.getSize().y / 2.f });





    // Simple camera offset: place world origin at screen center
    const sf::Vector2f screenCenter(Window_Width/2, Window_Height/2);


    // Simulation parameters
    const float timeStep = 1.0f / Window_Max_FPS;
    const int subSteps = 16;


    // --- Mouse drag state ---
    bool dragging = false;
    bool just_released(false);
    b2Vec2 dragOffset = { 0,0 };


    // Main loop
    while (window.isOpen())
    {
        while (auto eventOpt = window.pollEvent())  // pollEvent() returns std::optional<sf::Event>
        {
            if (!eventOpt) continue;
            sf::Event& event = *eventOpt;                 // dereference the optional

            if (event.is<sf::Event::Closed>())       // check if the event is "Closed"
                window.close();
            
            // --- Mouse pressed ---
            if (event.is<sf::Event::MouseButtonPressed>())
            {
                auto mouseEvent = event.getIf<sf::Event::MouseButtonPressed>();
                if (mouseEvent->button == sf::Mouse::Button::Left)
                {
                    sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
                    b2Vec2 mouseWorld({ mousePixel.x / PIXELS_PER_METER, mousePixel.y / PIXELS_PER_METER });
                    b2Vec2 offset({ float(Window_Width)/PIXELS_PER_METER/2, float(Window_Height)/PIXELS_PER_METER/2});
                    // check if click inside circle
                    b2Vec2 bodyPos = b2Body_GetPosition(phy.get_body_ID(BID[0])) + offset;
                    b2Vec2 diff = mouseWorld - bodyPos;
                    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                    cout << bodyPos.x    << ", " << bodyPos.y    << endl;
                    cout << mouseWorld.x << ", " << mouseWorld.y << endl;
                    if (dist <= Circle_radios) {
                        dragging = true;
                        dragOffset = diff;
                        b2Body_SetAwake(phy.get_body_ID(BID[0]), true);
                    }
                }
            }

            if (event.is<sf::Event::MouseButtonReleased>())
            {
                auto mouseEvent = event.getIf<sf::Event::MouseButtonReleased>();
                if (mouseEvent->button == sf::Mouse::Button::Left)
                {
                    // stop dragging
                    dragging = false;
                    just_released = true;
                }
            }
        }


        // --- Dragging logic ---
        if (dragging)
        {
            b2Vec2 offset({ float(Window_Width) / PIXELS_PER_METER / 2, float(Window_Height) / PIXELS_PER_METER / 2 });
            sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
            b2Vec2 mouseWorld({ mousePixel.x / PIXELS_PER_METER, mousePixel.y / PIXELS_PER_METER });
            b2Vec2 target = mouseWorld - dragOffset - offset;

            // apply force toward mouse
            b2Vec2 pos = b2Body_GetPosition(phy.get_body_ID(BID[0]));
            //b2Vec2 vel = b2Body_GetLinearVelocity(phy.get_body_ID(BID[0]));
            b2Vec2 vel = 20.0f * (target - pos);
            b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), vel);
            b2Body_SetAwake(phy.get_body_ID(BID[0]), true);
        }
        if (just_released)
        {
            just_released = false;
            b2Vec2 vel = b2Body_GetLinearVelocity(phy.get_body_ID(BID[0]));
            b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), { vel.x / 5, vel.y / 5 });
            b2Body_SetAwake(phy.get_body_ID(BID[0]), true);
        }


        // Step Box2D world
        b2World_Step(phy.get_world_ID(), timeStep, subSteps);


        
        b2Vec2 vel = b2Body_GetLinearVelocity(phy.get_body_ID(BID[1]));
        b2Body_SetLinearVelocity(phy.get_body_ID(BID[1]), { vel.x * damping_factor, vel.y });

        vel = b2Body_GetLinearVelocity(phy.get_body_ID(BID[0]));
        b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), { vel.x * damping_factor, vel.y });
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
        {
            b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), { -10.0f, vel.y });
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
        {
            b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), {  10.0f, vel.y });
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space))
        {
            b2Body_SetLinearVelocity(phy.get_body_ID(BID[0]), { vel.x, -10.0f });
        }





        // Ground position:
        b2Vec2 gpos = b2Body_GetPosition(phy.get_body_ID(GID));
        groundRect.setPosition({ screenCenter.x + gpos.x * PIXELS_PER_METER,
            screenCenter.y + gpos.y * PIXELS_PER_METER });
        // groundRect rotation should be zero for our simple example

        

        // Query body position & rotation (v3 API)
        b2Vec2 r1pos = b2Body_GetPosition(phy.get_body_ID(BID[0]));      // in meters
        b2Rot  r1rot = b2Body_GetRotation(phy.get_body_ID(BID[0]));       // rotation struct
        float  r1angleRad = b2Rot_GetAngle(r1rot);                          // radians
        // Convert to SFML coordinates (pixels)
        float r1x_px = screenCenter.x + r1pos.x * PIXELS_PER_METER;
        float r1y_px = screenCenter.y + r1pos.y * PIXELS_PER_METER;
        rect1.setPosition({ r1x_px, r1y_px });
        sf::Angle r1ang = sf::radians(r1angleRad);
        rect1.setRotation(r1ang * 180.0f / 3.14159265f);



        // Query body position & rotation (v3 API)
        b2Vec2 cpos = b2Body_GetPosition(phy.get_body_ID(BID[1])) + Circle_offset;      // in meters
        b2Rot  crot = b2Body_GetRotation(phy.get_body_ID(BID[1]));       // rotation struct
        float  cangleRad = b2Rot_GetAngle(crot);                          // radians
        // Convert to SFML coordinates (pixels)
        float cx_px = screenCenter.x + cpos.x * PIXELS_PER_METER;
        float cy_px = screenCenter.y + cpos.y * PIXELS_PER_METER;
        circ.setPosition({ cx_px, cy_px });
        sf::Angle cang = sf::radians(cangleRad);
        circ.setRotation(cang * 180.0f / 3.14159265f);



        // Query body position & rotation (v3 API)
        b2Vec2 r2pos = b2Body_GetPosition(phy.get_body_ID(BID[2]));      // in meters
        b2Rot  r2rot = b2Body_GetRotation(phy.get_body_ID(BID[2]));       // rotation struct
        float  r2angleRad = b2Rot_GetAngle(r2rot);                          // radians
        // Convert to SFML coordinates (pixels)
        float r2x_px = screenCenter.x + r2pos.x * PIXELS_PER_METER;
        float r2y_px = screenCenter.y + r2pos.y * PIXELS_PER_METER;
        rect2.setPosition({ r2x_px, r2y_px });
        sf::Angle r2ang = sf::radians(r2angleRad);
        rect2.setRotation(r2ang * 180.0f / 3.14159265f);


        // Render
        window.clear(sf::Color::Black);
        window.draw(groundRect);
        window.draw(rect1);
        window.draw(circ);
        window.draw(rect2);
        window.display();
    }

    // Cleanup — destroy world (this also removes bodies/shapes)
    phy.~physics();

    return 0;
}


#endif //__USE_BULLET__

#endif // __TEST_CASE__

