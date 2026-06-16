#pragma once

#include <torch/torch.h>
#include "Utilities.h"
#include "Constants.h"




// =======================
// Q-Network
// =======================
struct QNetworkImpl : torch::nn::Module {
    torch::nn::Linear fc1{ nullptr };
    torch::nn::Linear fc2{ nullptr };
    torch::nn::Linear fc3{ nullptr };
    torch::nn::Linear fc4{ nullptr };

    QNetworkImpl(int input_dim, int output_dim);

    torch::Tensor forward(torch::Tensor x);
};
TORCH_MODULE(QNetwork);


// =======================
// Replay Buffer
// =======================
class ReplayBuffer {
public:
    ReplayBuffer(size_t capacity, int mrs);

    void push(const Transition& t);

    bool can_sample();

    std::vector<Transition> sample(size_t batch_size);

    size_t size() const { return buffer_.size(); }

private:
    int     min_required_samples;
    std::vector<Transition> buffer_;
    size_t capacity_;
    size_t pos_ = 0;
    std::mt19937 rng{ std::random_device{}() };
};


// =======================
// DQN Agent
// =======================
class DQNAgent {
public:
    DQNAgent(int state_size, int action_size,
        float lr, float epsilon, size_t buffer_capacity, int mrs);

    // Copy the network's weigths 
    void copy_weights();
    
    void soft_update(float tua);

    // Convert State_Info -> torch::Tensor
    torch::Tensor state_to_tensor(const State_Info& s);

    // =======================
    // Action Selection
    // =======================
    Action_Info select_action(const State_Info& state, Action_Selection_mode mode);


    // =======================
    // Store Transition
    // =======================
    void remember(const State_Info& state, const Action_Info& action,
        float reward, const State_Info& next_state, bool done);

    void remember(Transition& Tr);

    // Converts Action_Info -> 16-element one-hot index
    int action_to_index(const Action_Info& action);


    // Converts index -> one-hot tensor
    torch::Tensor index_to_onehot(int idx, int n = 16);


    // =======================
    // Does Reply buffer has enough samples
    // =======================
    bool Reply_Buffer_Can_Sample();


    // =======================
    // Training (Batch Update) - FIXED FOR MULTI-ACTION
    // =======================
    void train_batch(size_t batch_size, float gamma = 0.99f);


    // =======================
    // Updating Epsilon and Learning Rate
    // =======================
    void Decay_Epsilon(float DecayVal);
    //void Decay_LearningRate(float DecayVal);

    // =======================
    // Saving and Loading The Network
    // =======================
    void Save(std::string name);
    bool Load(std::string name);


    void Set_epsilon(float eps);

private:
    int state_size;
    int action_size;
    float epsilon;
    QNetwork q_network = nullptr;
    QNetwork target_network = nullptr;
    std::unique_ptr<torch::optim::Adam> optimizer;
    ReplayBuffer replay_buffer;
    std::mt19937 rng{ std::random_device{}() };
};









