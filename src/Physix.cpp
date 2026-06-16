#include "Physix.h"


physics::physics(b2Vec2 grav)
{
	this->IDcntr = -1;
	this->worldDef = b2DefaultWorldDef();				// initialize with defaults
	this->worldDef.gravity = grav;
	this->world = b2CreateWorld(&worldDef);				// assigning a unique ID to the world
}


physics::~physics()
{
	b2DestroyWorld(this->world);
}


b2WorldId physics::get_world_ID()
{
	return world;
}


b2BodyId physics::get_body_ID(int which_body_ID)
{
	bool found(false);
	unsigned int i;
	for (i = 0; i < IDs.size(); i++)
	{
		if (IDs[i] == which_body_ID)
		{
			found = true;
			break;
		}
	}
	if (!found)
	{
		std::cerr << "Fatal error: wrong body ID." << std::endl;
		abort();
	}
	return *Bodies[i];
}


int  physics::add_body_and_shape(b2BodyDef& BodyDef, b2Polygon & BodyPoly, b2ShapeDef & BodyShapeDef)
{
	this->IDcntr = this->IDcntr + 1;

	b2BodyId bodyID = b2CreateBody(this->world, &BodyDef);
	b2ShapeId BoxShape = b2CreatePolygonShape(bodyID, &BodyShapeDef, &BodyPoly);
	(void)BoxShape;


	Bodies.push_back(std::make_unique<b2BodyId>(bodyID));
	Shapes.push_back(std::make_unique<b2ShapeId>(BoxShape));
	IDs.push_back(IDcntr);

	return IDcntr;
}

int physics::add_body_and_shape(b2BodyDef& BodyDef, b2Circle& BodyPoly, b2ShapeDef& BodyShapeDef)
{
	this->IDcntr = this->IDcntr + 1;

	b2BodyId bodyID = b2CreateBody(this->world, &BodyDef);
	b2ShapeId BoxShape = b2CreateCircleShape(bodyID, &BodyShapeDef, &BodyPoly);
	(void)BoxShape;
	

	Bodies.push_back(std::make_unique<b2BodyId>(bodyID));
	Shapes.push_back(std::make_unique<b2ShapeId>(BoxShape));
	IDs.push_back(IDcntr);

	return IDcntr;
}


bool physics::is_the_world_available()
{
	int ret(0);
	if (B2_IS_NULL(world)) {
		std::cerr << "Failed to create Box2D world\n";
		return false;
	}

	return true;
}
