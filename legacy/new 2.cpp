// main.cpp
// SFML + Box2D (v3.x) minimal example
// Requires: #include "box2d/box2d.h" from Box2D 3.x and SFML includes

#include "Shape.h"
#include "Physix.h"
#include "Utilities.h"
#include "Shader.h"
#include "Constants.h"
#include "Lunar_Lander_Environment.h"
#include "NN.h"


#ifdef _WIN32
#include <windows.h>
extern "C" {
    __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif






int main()
{
    // ----------------------------
    // Setup random generator
    // ----------------------------
    std::random_device rd;
    std::mt19937 gen(rd());

    // ----------------------------
    // Create Environment
    // ----------------------------
    LLEnv* Env = new LLEnv(gen, 120, Simulation_TimeStep, 0.02f, 0.035f, 100.0f);
    Env->Initialize();
    Env->create_window("New Environment");
    Env->Set_Engine_Angle(45.0f, -45.0f);
    Env->Add_to_fule(100);
    Action_Info         Act_info;
    Observation_Info    Obs_info;

    // ----------------------------
    // 3. DQN Agent setup
    // ----------------------------
    DQNAgent agent(STATE_DIM, N_ACTIONS, 1e-3, 0.1f, 500000);
    size_t batch_size = 32;
    float gamma = 0.99f;
    int max_episodes = 50000;



    bool Force_applied_left(false);
    bool Force_applied_right(false);
    bool space_pressed(false);


    // --- Main Loop ---
    while (Env->Active_window().isOpen())
    {
        while (auto eventOpt = Env->Active_window().pollEvent())  // pollEvent() returns std::optional<sf::Event>
        {
            if (!eventOpt) continue;
            sf::Event& event = *eventOpt;                 // dereference the optional

            if (event.is<sf::Event::Closed>())       // check if the event is "Closed"
                Env->Active_window().close();
        }

        
        Act_info.RE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
        Act_info.LE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
        Act_info.RE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space);
        Act_info.LE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space);
        
        Env->Take_Action(Act_info);
        Env->Step();
        Env->Get_Observation(Obs_info);

        std::cout << Obs_info.state.fule_left << std::endl;

        Env->Clear_currecnt_Window();
        Env->draw_all();
        Env->Render();
    }
    // --- Cleanup ---
    delete Env;
    return 0;
}

