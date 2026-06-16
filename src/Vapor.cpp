#include "Vapor.h"
#include "Constants.h"



EngineVapor::EngineVapor(btDiscreteDynamicsWorld* world)
    : world(world)
{
    shape = new btSphereShape(0.05f); // small physical radius

    puff.setFillColor(sf::Color(255, 255, 255, 180)); // white semi-transparent
    puff.setRadius(Vapor_radius * SCALE);
    puff.setOrigin({ Vapor_radius * SCALE, Vapor_radius * SCALE });
    exhaustDir = btVector3(0.0f, -0.125f, 0.0f);
}

EngineVapor::~EngineVapor()
{
    // Delete all particle rigid bodies
    for (auto& p : particles)
    {
        if (p.body)
        {
            world->removeRigidBody(p.body);  // Remove from Bullet world
            delete p.body->getMotionState(); // Delete motion state
            delete p.body;                   // Delete rigid body
            p.body = nullptr;
        }
    }
    particles.clear();

    // Delete the collision shape
    if (shape)
    {
        delete shape;
        shape = nullptr;
    }
}

void EngineVapor::setEmitter(const btVector3& pos)
{
    emitterPos = pos;
}

void EngineVapor::emit(int count, int speed_multiplier)
{
    if (count != 0)
    {
        count = count + ((speed_multiplier - 1) * Vapor_cnt_speed_coef);
        for (int i = 0; i < count; ++i) {
            btTransform t;
            t.setIdentity();
            t.setOrigin(emitterPos);

            btScalar mass = Vapor_mass;
            btVector3 inertia(0, 0, 0);
            shape->calculateLocalInertia(mass, inertia);

            btDefaultMotionState* motion = new btDefaultMotionState(t);
            btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
            btRigidBody* body = new btRigidBody(info);

            // Each vapor particle floats upward slightly
            body->setGravity(btVector3(0, 0, 0));
            body->setDamping(0.3f, 0.3f);
            body->setFriction(0.0f);
            body->setRestitution(0.0f);
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

            // ---  Engine exhaust impulse ---
            float thrust = Vapor_thrust_base + ((float(rand()) / RAND_MAX) * Vapor_thrust_vary);  // stronger base speed
            thrust = thrust + speed_multiplier * Vapor_thrust_base * Vapor_thrust_speed_coef;
            float spread = Vapor_angular_spread_range; // angular spread of exhaust plume
            btVector3 randomDir = exhaustDir +
                btVector3(
                    (float(rand()) / RAND_MAX - 0.5f) * spread,
                    (float(rand()) / RAND_MAX - 0.5f) * spread,
                    0.0f);
            randomDir.normalize();
            btVector3 velocity = randomDir * thrust;

            body->setLinearVelocity(velocity);

            world->addRigidBody(body);

            VaporParticle p;
            p.body = body;
            p.lifetime = Vapor_life_time_base + ((float(rand()) / RAND_MAX) * Vapor_life_time_vary);
            p.lifetime = p.lifetime + speed_multiplier * (Vapor_life_time_base * Vapor_life_time_speed_coef);
            p.opacity = 255.f;
            p.size = Vapor_radius;
            particles.push_back(p);
        }
    }
}

void EngineVapor::update(float dt)
{
    for (auto& p : particles) {
        p.lifetime -= dt;
        p.opacity = std::max(0.f, 255.f * (p.lifetime / Vapor_life_time_base));
        // vapor expands slightly
        p.size += Vapor_radius_expantion_rate * dt;
    }

    // Remove expired particles
    particles.erase(std::remove_if(particles.begin(), particles.end(),
        [&](VaporParticle& p)
        {
            if (p.lifetime <= 0.f) {
                world->removeRigidBody(p.body);
                delete p.body->getMotionState();
                delete p.body;
                return true;
            }
            return false;
        }), particles.end());
}

void EngineVapor::draw(sf::RenderWindow& window)
{
    for (auto& p : particles) {
        btTransform t;
        p.body->getMotionState()->getWorldTransform(t);
        btVector3 pos = t.getOrigin();

        // Convert from Bullet (meters) to SFML (pixels)
        sf::Vector2f screenPos = toSFML(pos);

        puff.setPosition(screenPos);
        sf::Color c = puff.getFillColor();
        c.a = static_cast<unsigned char>(p.opacity);
        puff.setFillColor(c);
        window.draw(puff);
    }
}

void EngineVapor::setDirection(const btVector3& dir)
{
    exhaustDir = dir;
}


