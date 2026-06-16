#include "NN.h"

QNetworkImpl::QNetworkImpl(int input_dim, int output_dim)
{
    fc1 = register_module("fc1", torch::nn::Linear(input_dim, 128));
    fc2 = register_module("fc2", torch::nn::Linear(128, 64));
    fc3 = register_module("fc3", torch::nn::Linear(64, 64));
    fc4 = register_module("fc4", torch::nn::Linear(64, output_dim));
}

torch::Tensor QNetworkImpl::forward(torch::Tensor x)
{
    auto h1 = fc1->forward(x);

    auto h2 = fc2->forward(torch::relu(h1));

    auto h3 = fc3->forward(torch::relu(h2));

    auto out = fc4->forward(torch::relu(h3));

    return out;
}




ReplayBuffer::ReplayBuffer(size_t capacity, int mrs)
    : capacity_(capacity),
    min_required_samples(mrs)
{
    
}

void ReplayBuffer::push(const Transition& t)
{
    if (buffer_.size() < capacity_)
        buffer_.push_back(t);
    else {
        buffer_[pos_] = t;
        pos_ = (pos_ + 1) % capacity_;
    }
}

bool ReplayBuffer::can_sample()
{
    return buffer_.size() >= min_required_samples;
}

std::vector<Transition> ReplayBuffer::sample(size_t batch_size)
{
    std::vector<Transition> batch;
    std::uniform_int_distribution<size_t> dist(0, buffer_.size() - 1);
    batch.reserve(batch_size);
    for (size_t i = 0; i < batch_size; ++i)
        batch.push_back(buffer_[dist(rng)]);
    return batch;
}




DQNAgent::DQNAgent(int state_size, int action_size, float lr, float epsilon, size_t buffer_capacity, int mrs)
    : state_size(state_size),
    action_size(action_size),
    epsilon(epsilon),
    replay_buffer(buffer_capacity, mrs)
{
    q_network = QNetwork(state_size, action_size);
    target_network = QNetwork(state_size, action_size);
    copy_weights();
    target_network->eval();

    optimizer = std::make_unique<torch::optim::Adam>(q_network->parameters(),
        torch::optim::AdamOptions(lr));
}

void DQNAgent::copy_weights()
{
    auto src_params = q_network->named_parameters();
    auto dest_params = target_network->named_parameters();
    torch::NoGradGuard no_grad;
    for (auto& item : src_params)
    {
        const auto& name = item.key();
        if (dest_params.contains(name))
            dest_params[name].copy_(item.value());
    }
}

void DQNAgent::soft_update(float tau)
{
    auto src_params = q_network->named_parameters();
    auto dest_params = target_network->named_parameters();
    torch::NoGradGuard no_grad;
    for (auto& item : src_params)
    {
        const auto& name = item.key();
        if (dest_params.contains(name))
        {
            // Soft update: target = tau*source + (1-tau)*target
            dest_params[name].mul_(1.0 - tau);
            dest_params[name].add_(item.value(), tau);
        }
    }
}

torch::Tensor DQNAgent::state_to_tensor(const State_Info& s)
{
    std::vector<float> vec = {
        floor(s.Lander_Pos_X   / STATE_PRECISION_POSITION) * STATE_PRECISION_POSITION,
        floor(s.Lander_Pos_Y   / STATE_PRECISION_POSITION) * STATE_PRECISION_POSITION,
        floor(s.Lander_LV_X    / STATE_PRECISION_SPEED   ) * STATE_PRECISION_SPEED,
        floor(s.Lander_LV_Y    / STATE_PRECISION_SPEED   ) * STATE_PRECISION_SPEED,
        floor(s.Lander_Ang_val / STATE_PRECISION_ANGLE   ) * STATE_PRECISION_ANGLE,
              s.Lander_Ang_sgn,
        floor(s.Lander_Ang_vel / STATE_PRECISION_ANGLE   ) * STATE_PRECISION_ANGLE
        //s.wind_speed_Direction_X,
        //s.wind_speed_Direction_Y,
        //s.Platform_Pos_X,
        //s.Platform_Pos_Y,
        //s.fule_left
        };
    return torch::tensor(vec).unsqueeze(0);
}

Action_Info DQNAgent::select_action(const State_Info& state, Action_Selection_mode mode)
{
    std::uniform_real_distribution<float> dist(0.0, 1.0);
    Action_Info action;

    float eps(epsilon);
    if (mode == Random_mode)
        eps = 1.0f;
    if (mode == Inferense_mode)
        eps = 0.0f;

    if (dist(rng) < eps)
    {
        action.RE_Active = dist(rng) < 0.5;
        action.LE_Active = dist(rng) < 0.5;
        action.RE_Boost = dist(rng) < 0.5;
        action.LE_Boost = dist(rng) < 0.5;
    }
    else
    {
        torch::NoGradGuard no_grad;
        auto input = state_to_tensor(state);
        auto q_values = q_network->forward(input).squeeze(0);
        int action_idx = q_values.argmax(0).item<int>();  // 0–15
        action.RE_Active = (action_idx / 1) % 2;
        action.LE_Active = (action_idx / 2) % 2;
        action.RE_Boost  = (action_idx / 4) % 2;
        action.LE_Boost  = (action_idx / 8) % 2;
    }

    return action;
}

void DQNAgent::remember(const State_Info& state, const Action_Info& action, float reward, const State_Info& next_state, bool done)
{
    replay_buffer.push({ state, action, reward, next_state, done });
}

void DQNAgent::remember(Transition& Tr)
{
    replay_buffer.push(Tr);
}

int DQNAgent::action_to_index(const Action_Info& action)
{
    int idx = 0;
    if (action.RE_Active) idx += 1;
    if (action.LE_Active) idx += 2;
    if (action.RE_Boost)  idx += 4;
    if (action.LE_Boost)  idx += 8;
    return idx; // 0..15
}

torch::Tensor DQNAgent::index_to_onehot(int idx, int n)
{
    auto t = torch::zeros({ n }, torch::kFloat32);
    t[idx] = 1.0f;
    return t;
}

bool DQNAgent::Reply_Buffer_Can_Sample()
{
    return replay_buffer.can_sample();
}

void DQNAgent::train_batch(size_t batch_size, float gamma)
{
    auto transitions = replay_buffer.sample(batch_size);

    std::vector<torch::Tensor> state_batch, next_state_batch;
    std::vector<int64_t> action_indices;
    std::vector<torch::Tensor> reward_batch, done_batch;

    for (auto& t : transitions)
    {
        state_batch.push_back(state_to_tensor(t.state).to(torch::kFloat32).detach());
        next_state_batch.push_back(state_to_tensor(t.next_state).to(torch::kFloat32).detach());

        int idx = action_to_index(t.action);
        action_indices.push_back(idx);

        reward_batch.push_back(torch::tensor({ t.reward }, torch::kFloat32));
        done_batch.push_back(torch::tensor({ t.done ? 1.0f : 0.0f }, torch::kFloat32));
    }

    auto device = q_network->parameters().front().device();

    auto states = torch::cat(state_batch, 0).to(device);       // [batch, state_dim]
    auto next_states = torch::cat(next_state_batch, 0).to(device);
    auto rewards = torch::cat(reward_batch, 0).unsqueeze(1).to(device); // [batch,1]
    auto dones = torch::cat(done_batch, 0).unsqueeze(1).to(device);     // [batch,1]

    auto actions = torch::tensor(action_indices, torch::kInt64).unsqueeze(1).to(device); // [batch,1]

    // Forward pass
    auto q_pred = q_network->forward(states);    // [batch,16]
    auto q_pred_taken = q_pred.gather(1, actions); // Q-value for the chosen action [batch,1]

    // Compute target Q-values
    torch::Tensor q_target;
    {
        torch::NoGradGuard no_grad;                          // grads disabled inside this scope
        auto q_next = target_network->forward(next_states); // [batch,16]
        auto q_next_max = std::get<0>(q_next.max(1, true)); // [batch,1]
        q_target = (rewards + (1 - dones) * gamma * q_next_max).detach();
    }

    // Loss
    auto loss = torch::mse_loss(q_pred_taken, q_target);
    optimizer->zero_grad();
    loss.backward();
    optimizer->step();


}

void DQNAgent::Decay_Epsilon(float DecayVal)
{
    epsilon = epsilon * DecayVal;
    if (epsilon < 1e-2)
        epsilon = 1e-2;
}

void DQNAgent::Save(std::string name)
{
    torch::save(q_network, name);
}

bool DQNAgent::Load(std::string name)
{
    if (!q_network)
        return false;
    torch::load(q_network, name);
    torch::load(target_network, name);
    return true;
}

void DQNAgent::Set_epsilon(float eps)
{
    epsilon = eps;
    Decay_Epsilon(1.0f);
}










