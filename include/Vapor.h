#pragma once

#include "Utilities.h"
#include "Constants.h"

class EngineVapor
{
public:
    EngineVapor(btDiscreteDynamicsWorld* world);
    ~EngineVapor();

    void setEmitter(const btVector3& pos);
    void emit(int count, int speed_multiplier);
    void update(float dt);
    void draw(sf::RenderWindow& window);
    void setDirection(const btVector3& dir);

private:
    btDiscreteDynamicsWorld* world;
    btCollisionShape* shape;
    std::vector<VaporParticle> particles;
    sf::CircleShape puff;

    btVector3 emitterPos;
    btVector3 exhaustDir;
};




