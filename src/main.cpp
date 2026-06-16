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
#include "SafeQueue.h"


#ifdef _WIN32
#include <windows.h>
extern "C" {
    __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif



//#define __TEST_CASE__
#define __RUN_ORIGINAL__




// Test Case
#ifdef __TEST_CASE__


int main()
{
    cv::Mat img(400, 400, CV_8UC3, cv::Scalar(0, 255, 0));
    cv::imshow("Test", img);
    cv::waitKey(0);
    return 0;
}



#endif // __TEST_CASE__


// Normal Function
#ifndef __TEST_CASE__
#ifndef __RUN_ORIGINAL__
#define __RUN_THREAD_BASED__
#endif//__RUN_ORIGINAL__


//  the original
#ifdef __RUN_ORIGINAL__

//#define __RUN_TRAINING__
//#define __RUN_SIMULATION__
//#define __RUN_MANULA__



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
    const float Agent_Starting_Learning_Rate(0.0005f);
    const int   Agent_Update_interval(1000);
    const int   batch_size(64);
    const float gamma(0.99f);
    const int   max_episodes(20000);
    const int   Env_Max_FPS(120);
    const float Env_Base_Fule_Consumption(0.0075f);
    const float Env_Normal_Fule_Consumption(0.03f);
    const float Env_Boosted_Fule_Consumption(0.045f);
    const float Env_Starting_Fule(100.0f);
    float       Agent_Starting_Epsilon(1.0f);
    const int   Agent_Reply_Buffer_capacity(250000);
    const int   Agent_Reply_Buffer_min_samples(10000);

    bool        Schedule_Simulation(false);
    bool        Schedule_Manual(false);
    bool        Doing_Simulation(false);
    bool        Doing_Manual(false);
    bool        Write_Video_En(true);

    LLEnv* Env = new LLEnv(
        gen,
        Env_Max_FPS,
        Simulation_TimeStep,
        Env_Base_Fule_Consumption,
        Env_Normal_Fule_Consumption,
        Env_Boosted_Fule_Consumption,
        Env_Starting_Fule);

    std::string str = "Something";
    Env->restart();
    Env->Set_Engine_Angle(45.0f, -45.0f);
    Env->Add_to_fule(100);
    Env->Disable_FPS_Limit();
    Action_Info         Act_info;
    Observation_Info    Obs_info;
    Observation_Info    next_obs;

    // ----------------------------
    // 3. DQN Agent setup
    // ----------------------------
    DQNAgent* agent = new DQNAgent(
        7,
        16,
        Agent_Starting_Learning_Rate,
        Agent_Starting_Epsilon,
        Agent_Reply_Buffer_capacity,
        Agent_Reply_Buffer_min_samples);


    if (agent->Load("Q_net"))
        std::cout << "Load Completed" << std::endl;
    else
        std::cout << "Couldn't Load" << std::endl;



    // ----------------------------
    // 4. filling the reply buffer
    // ----------------------------
    std::cout << "Filling the reply buffer:  " << std::endl;
    int steps(0);
    while (!agent->Reply_Buffer_Can_Sample())
    {
        Env->restart();
        Env->Add_to_fule(Env_Starting_Fule);
        Env->Get_Observation(Obs_info);
        int step_count = 0;
        while (!Obs_info.Done)
        {
            step_count++;
            Action_Info action = agent->select_action(Obs_info.state, Random_mode);
            Env->Take_Action(action);
            Env->Step();
            Env->Get_Observation(next_obs);
            agent->remember(Obs_info.state, action, next_obs.Reward, next_obs.state, next_obs.Done);
            Obs_info = next_obs;
        }
        steps += step_count;
        std::cout << "\r\tNumber Of Inserted Transitions: " << steps;
    }
    std::cout << std::endl << "\rDone!" << std::endl << std::endl;


    // ----------------------------
    // 5. Main training loop
    // ----------------------------
    std::cout << "Training: " << std::endl;
    size_t total_steps(0);
    bool skip(false);
    unsigned int video_cntr(0);
    for (int episode = 0; episode <= max_episodes; ++episode)
    {
        skip = false;
        float total_reward = 0.0f;
        int step_count = 0;
        Schedule_Simulation = false;
        Schedule_Manual = false;

        if ((!Doing_Simulation) && (!Doing_Manual))
            std::cout <<
                "Episode: " << std::setw(6) << episode + 1 <<
                " | Steps: " << std::setw(6) << step_count <<
                " | Total Reward: " << std::setw(14) << total_reward;

        do
        {
            Env->restart();
            Env->Add_to_fule(Env_Starting_Fule);
            Env->Get_Observation(Obs_info);
        } while (Obs_info.Done);


        if (Doing_Simulation && Write_Video_En)
        {
            video_cntr++;
            std::string name = "Simulation_No" + std::to_string(video_cntr) + ".mp4";
            Env->Create_Video(name);
        }


        while (!Obs_info.Done)
        {
            Schedule_Simulation |= sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl) && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q);
            Schedule_Manual     |= sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl) && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
            skip                 = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl) && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E);

            Env->Polling_Window_Event();

            // --- 4a. Select action ---
            if (Doing_Simulation)
                Act_info = agent->select_action(Obs_info.state, Inferense_mode);
            else if (Doing_Manual)
            {
                Act_info.RE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)       ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
                Act_info.LE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)       ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
                Act_info.RE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right)   ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
                Act_info.LE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left)    ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
            }
            else
                Act_info = agent->select_action(Obs_info.state, Train_mode);

            // --- 4b. Apply action ---
            Env->Take_Action(Act_info);
            Env->Step();

            // --- 4c. Render environment ---
            if (Doing_Simulation || Doing_Manual)
            {
                if (!Env->Active_window_isOpen())
                    Env->Create_Window(str);
                Env->Show_Window();
                Env->Clear_Window();
                Env->draw_all();
                Env->Render();
                if (Write_Video_En)
                    Env->Add_a_Frame();
            }

            // --- 4d. Get next observation ---
            Env->Get_Observation(next_obs);
            if (Doing_Simulation || Doing_Manual)
                Env->Print_Observation_info(next_obs, Act_info);

            // --- 4e. Store transition ---
            agent->remember(Obs_info.state, Act_info, next_obs.Reward, next_obs.state, next_obs.Done);

            // --- 4f. Train agent ---
            agent->train_batch(batch_size, gamma);


            total_reward += next_obs.Reward;
            step_count++;
            total_steps++;
            Obs_info = next_obs;

            if (total_steps % Agent_Update_interval == 0)
                agent->copy_weights();

            if (!(Doing_Simulation || Doing_Manual) && (step_count % 100 == 0))
            {
                for (int i = 0; i < 37; i++)
                    std::cout << "\b";
                std::cout << std::setw(6) << step_count << " | Total Reward: " << std::setw(14) << total_reward;
            }

            if (skip || (step_count > 2500))
                break;
        }

        if (Doing_Simulation || Doing_Manual)
        {
            if (Write_Video_En)
                Env->Release_video();
            std::cout << std::endl;
        }

        Doing_Simulation = Schedule_Simulation;
        Doing_Manual     = Schedule_Manual;
        Env->Hide_Window();
        agent->Decay_Epsilon(0.975f);
        std::cout << "\rEpisode: " << std::setw(6) << episode + 1
            << " | Steps: " << std::setw(6) << step_count
            << " | Total Reward: " << std::setw(14) << total_reward << std::endl;

        if (episode % 10 == 0)
            agent->Save("Q_net");

        if (episode % 250 == 0)
        {
            Agent_Starting_Epsilon = Agent_Starting_Epsilon * 0.9f;
            agent->Set_epsilon(Agent_Starting_Epsilon);
        }
    }


    // ----------------------------
    // 6. Main  loop
    // ----------------------------
    Env->Create_Window(str);
    Env->Set_FPS_limit(120);
    while (Env->Active_window_isOpen())
    {
        do
        {
            Env->restart();
            Env->Add_to_fule(Env_Starting_Fule);
            Env->Get_Observation(Obs_info);
        } while (Obs_info.Done);

        Env->Create_Window(str);


        while (!Obs_info.Done)
        {
            Env->Polling_Window_Event();


            Act_info.RE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)       ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
            Act_info.LE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)       ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
            Act_info.RE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right)   ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
            Act_info.LE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left)    ||  sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);

            Env->Take_Action(Act_info);
            Env->Step();
            Env->Get_Observation(Obs_info);

            std::cout << Obs_info.state.fule_left << std::endl;

            Env->Clear_Window();
            Env->draw_all();
            Env->Render();
        }
    }
    // --- Cleanup ---
    delete Env;
    return 0;
}







#endif // __RUN_ORIGINALS__
#ifdef __RUN_THREAD_BASED__

#define __RUN_THREAD_V1__
//#define __RUN_THREAD_V2__
//#define __RUN_THREAD_V3__


//  Thread V3 Working 
#ifdef __RUN_THREAD_V3__
std::atomic<bool>   stop_Training     = false;
std::atomic<bool>   Save_en           = false;
std::atomic<bool>   Eps_decay_en      = false;
std::atomic<bool>   Train_Them_all    = false;
std::atomic<bool>   Done_Filling      = false;
std::atomic<size_t> Train_step        = 0;


void Training_Thread(DQNAgent* agent, std::shared_ptr<SafeQueue<Transition>>  replay_queue)
{
    int cntr(0);
    // ----------------------------
    // 1. filling the reply buffer
    // ----------------------------
    std::cout << std::endl << "Filling the reply buffer:  " << std::endl;
    Transition  Tr;
    while (!agent->Reply_Buffer_Can_Sample())
    {
        replay_queue->wait_and_pop(Tr);
        agent->remember(Tr.state, Tr.action, Tr.reward, Tr.next_state, Tr.done);
        cntr++;
        Train_step = cntr;
    }
    std::cout << std::endl << "Filling the reply buffer Done!" << std::endl << std::endl;
    Done_Filling = true;

    // ----------------------------
    // 2. Training loop
    // ----------------------------
    std::cout << std::endl << "Training: " << std::endl;
    while (!stop_Training)
    {
        replay_queue->wait_and_pop(Tr);
        cntr++;
        Train_step = cntr;

        // --- 4.a Store transition ---
        agent->remember(Tr);
        
        // --- 2.b Train agent ---
        agent->train_batch(batch_size, gamma);
        Train_step++;

        if (Train_step % Agent_Update_interval == 0)
            agent->copy_weights();

        if (Eps_decay_en)
        {
            agent->Decay_Epsilon(0.99f);
            Eps_decay_en = false;
        }

        if (Save_en)
        {
            agent->Save("Q_net");
            Save_en = false;
        }
    }
}

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
    const int   Env_Max_FPS(120);
    const float Env_Base_Fule_Consumption(0.001f);
    const float Env_Normal_Fule_Consumption(0.03f);
    const float Env_Boosted_Fule_Consumption(0.045f);
    const float Env_Starting_Fule(100.0f);

    LLEnv* Env = new LLEnv(
        gen,
        Env_Max_FPS,
        Simulation_TimeStep,
        Env_Base_Fule_Consumption,
        Env_Normal_Fule_Consumption,
        Env_Boosted_Fule_Consumption,
        Env_Starting_Fule);

    std::string str = "Something";
    Env->restart();
    Env->Set_Engine_Angle(45.0f, -45.0f);
    Env->Add_to_fule(100);
    Env->Disable_FPS_Limit();

    Action_Info         Act_info;
    Observation_Info    Obs_info;
    Observation_Info    next_obs;
    bool                Schedule_Simulation(false);
    bool                Schedule_Manual(false);
    bool                Doing_Simulation(false);
    bool                Doing_Manual(false);

    // ----------------------------
    // 3. DQN Agent setup
    // ----------------------------
    const int   Agent_Reply_Buffer_capacity(500000);
    const int   Agent_Reply_Buffer_min_samples(50000);

    DQNAgent* agent = new DQNAgent(
        STATE_DIM,
        N_ACTIONS,
        Agent_Starting_Learning_Rate,
        Agent_Starting_Epsilon,
        Agent_Reply_Buffer_capacity,
        Agent_Reply_Buffer_min_samples);

    if (agent->Load("Q_net"))
        std::cout << "Load Completed" << std::endl;
    else
        std::cout << "Couldn't Load" << std::endl;

    // ----------------------------
    // 4. Reply Buffer
    // ----------------------------
    auto replay_queue = std::make_shared<SafeQueue<Transition>>();


    // ----------------------------
    // 5. Starting Thread
    // ----------------------------
    std::thread Train_thread(Training_Thread, agent, replay_queue);


    // ----------------------------
    // 6. Simulation The Environment
    // ----------------------------
    size_t generate_step(0);
    size_t total_steps(0);
    for (int episode = 0; episode <= max_episodes; ++episode)
    {
        float total_reward = 0.0f;
        int step_count = 0;
        Schedule_Simulation = false;
        Schedule_Manual = false;

        do
        {
            Env->restart();
            Env->Add_to_fule(Env_Starting_Fule);
            Env->Get_Observation(Obs_info);
        } while (Obs_info.Done);



        while (!Obs_info.Done)
        {
            Schedule_Simulation |= sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl)  && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S);
            Schedule_Manual     |= sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl)  && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::R);

            Env->Polling_Window_Event();

            // --- 4a. Select action ---
            if (Doing_Simulation)
                Act_info = agent->select_action(Obs_info.state, Inferense_mode);
            else if (Doing_Manual)
            {
                Act_info.RE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)       || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
                Act_info.LE_Active = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)       || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);
                Act_info.RE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right)   || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
                Act_info.LE_Boost  = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left)    || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
            }
            else
                Act_info = agent->select_action(Obs_info.state, Train_mode);

            // --- 4b. Apply action ---
            Env->Take_Action(Act_info);
            // --- 4c. Advances ---
            Env->Step();
            // --- 4d. Get next observation ---
            Env->Get_Observation(next_obs);


            // --- 4e. Render environment ---
            if (Doing_Simulation || Doing_Manual)
            {
                if (!Env->Active_window_isOpen())
                    Env->Create_Window(str);
                Env->Show_Window();
                Env->Clear_Window();
                Env->draw_all();
                Env->Render();
                Env->Print_Observation_info(next_obs, Act_info);
            }

            

            // --- 4f. Send the Transition to the training thread ---
            replay_queue->push({ Obs_info.state, Act_info, next_obs.Reward, next_obs.state, next_obs.Done });
            generate_step++;
            
            total_reward += next_obs.Reward;
            step_count++;
            total_steps++;
            Obs_info = next_obs;
        }

        if (Doing_Simulation || Doing_Manual)
            std::cout << std::endl;

        Doing_Simulation = Schedule_Simulation;
        Doing_Manual = Schedule_Manual;
        Env->Hide_Window();
        Eps_decay_en = true;
        

        std::cout
            << "\rEpisode:     "    << std::setw(6)     << episode + 1
            << " | Steps: "         << std::setw(6)     << step_count
            << " | Total Reward: "  << std::setw(14)    << total_reward
            << " | Step Trained: (" << std::setw(10)    << generate_step
            << ", "                 << std::setw(10)    << Train_step 
            << ") " << std::endl;

        if (episode % 10 == 0)
            Save_en = true;

        
        while ((generate_step - Train_step) > 20)
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    stop_Training = true;
    if (Train_thread.joinable()) Train_thread.join();

    return 0;
}
#endif //__RUN_THREAD_V3__




//  the simple thread
#ifdef __RUN_THREAD_V1__
std::atomic<bool> training_done = false;
std::atomic<bool> stop_training = false;
std::atomic<bool> environment_ready = false;


void TrainAgent(LLEnv* Env, DQNAgent* agent, int max_episodes, int Agent_Update_interval, int Env_Show_Progress_Interval)
{
    Observation_Info Obs_info, next_obs;
    const size_t batch_size = 32;
    const float gamma = 0.99f;

    size_t total_steps(0);

    for (int episode = 0; episode <= max_episodes && !stop_training; ++episode)
    {
        Env->restart();
        Env->Add_to_fule(100.0f);
        Env->Get_Observation(Obs_info);

        float total_reward = 0.0f;
        int step_count = 0;

        while (!Obs_info.Done && !stop_training)
        {
            // --- Select & apply action ---
            Action_Info action = agent->select_action(Obs_info.state, Train_mode);
            Env->Take_Action(action);
            Env->Step();

            // --- Get next observation ---
            Env->Get_Observation(next_obs);

            // --- Store transition ---
            agent->remember(Obs_info.state, action, next_obs.Reward, next_obs.state, next_obs.Done);

            // --- Train network ---
            agent->train_batch(batch_size, gamma);

            total_reward += next_obs.Reward;
            step_count++;
            total_steps++;
            Obs_info = next_obs;

            agent->Decay_Epsilon(0.9999f);
            if (total_steps % Agent_Update_interval == 0)
                agent->copy_weights();
        }

        std::cout << "Episode " << episode + 1
            << " | Steps: " << step_count
            << " | Total Reward: " << total_reward << std::endl;

        if (episode % 10 == 0)
            agent->Save("Q_net");
    }

    training_done = true;
    std::cout << " Training finished!" << std::endl;
}

int main()
{
    // --- Random generator ---
    std::random_device rd;
    std::mt19937 gen(rd());

    // --- Environment ---
    const int Env_Show_Progress_Interval = 1;
    LLEnv* Env = new LLEnv(gen, 120, Simulation_TimeStep, 0.005f, 0.04f, 0.07f, 100.0f);
    std::string str = "Lunar Lander";
    Env->restart();
    Env->Create_Window(str);
    Env->Set_Engine_Angle(45.0f, -45.0f);
    Env->Add_to_fule(100.0f);


    // --- Agent ---
    const int Agent_Update_interval = 1000;
    DQNAgent agent(STATE_DIM, N_ACTIONS, 0.001f, 1.0f, 500000, 5000);


    // --- Start training in a separate thread ---
    std::thread training_thread(TrainAgent, Env, &agent, 2000, Agent_Update_interval, Env_Show_Progress_Interval);

    // --- SFML Rendering loop (main thread) ---
    Observation_Info Obs_info;
    Action_Info Act_info;

    while (Env->Active_window().isOpen())
    {
        Env->Polling_Window_Event();

        Env->Take_Action(Act_info);
        Env->Step();
        Env->Get_Observation(Obs_info);

        Env->Clear_Window();
        Env->draw_all();
        Env->Render();

        // Optional: show training progress
        if (training_done)
            break;
    }

    stop_training = true;
    if (training_thread.joinable()) training_thread.join();

    delete Env;
    return 0;
}
#endif // __RUN_THREAD_V3__




//  New Thread V2
#ifdef __RUN_THREAD_V3__
std::atomic<bool> stop_all = false;

void EnvThread(LLEnv* Env, DQNAgent* agent, SafeQueue<Transition>* replay_queue)
{
    Observation_Info Obs, Next;
    while (!stop_all)
    {
        Env->restart();
        Env->Add_to_fule(100.0f);
        Env->Get_Observation(Obs);

        while (!Obs.Done && !stop_all)
        {
            Action_Info action = agent->select_action(Obs.state, Train_mode);
            Env->Take_Action(action);
            Env->Step();
            Env->Get_Observation(Next);

            Transition t{ Obs.state, action, Next.Reward, Next.state, Next.Done };
            replay_queue->push(t);
            Obs = Next;
        }
    }
}

void TrainThread(DQNAgent* agent, SafeQueue<Transition>* replay_queue)
{
    const float gamma = 0.99f;
    const size_t batch_size = 32;
    const int min_samples = 5000;

    std::vector<Transition> local_buffer;

    while (!stop_all)
    {
        Transition t;
        while (replay_queue->try_pop(t))
        {
            agent->remember(t.state, t.action, t.reward, t.next_state, t.done);
        }

        if (agent->Reply_Buffer_Can_Sample())
        {
            agent->train_batch(batch_size, gamma);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

int main()
{
    std::random_device rd;
    std::mt19937 gen(rd());

    LLEnv Env(gen, 120, Simulation_TimeStep, 0.005f, 0.04f, 0.07f, 100.0f);
    DQNAgent agent(STATE_DIM, N_ACTIONS, 0.001f, 1.0f, 500000, 5000);

    SafeQueue<Transition> replay_queue;

    std::thread env_thread(EnvThread, &Env, &agent, &replay_queue);
    std::thread train_thread(TrainThread, &agent, &replay_queue);

    // --- Optional rendering loop ---
    std::string title = "Lunar Lander";
    Env.Create_Window(title);
    Observation_Info info;
    while (Env.Active_window().isOpen()) {
        Env.Polling_Window_Event();
        Env.Get_Observation(info);
        Env.Clear_Window();
        Env.draw_all();
        Env.Render();
    }

    stop_all = true;
    env_thread.join();
    train_thread.join();
    return 0;
}
#endif // __RUN_THREAD_V2__




#endif // __RUN_THREAD_BASED__






#endif __TEST_CASE__


