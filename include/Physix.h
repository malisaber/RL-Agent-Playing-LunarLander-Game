#pragma once

#include "Utilities.h"

class physics
{
	b2WorldId world;
	b2WorldDef worldDef;

	int IDcntr;
	std::vector<int> IDs;
	std::vector<std::unique_ptr<b2BodyId>>   Bodies;
	std::vector<std::unique_ptr<b2ShapeId>>  Shapes;



public:
	physics(b2Vec2 grav);
	~physics();

	b2WorldId get_world_ID();
	b2BodyId  get_body_ID(int which_body_ID);


	int  add_body_and_shape(b2BodyDef& BodyDef, b2Polygon& BodyPoly, b2ShapeDef& BodyShapeDef);
	int  add_body_and_shape(b2BodyDef& BodyDef, b2Circle&  BodyPoly, b2ShapeDef& BodyShapeDef);

	

	bool is_the_world_available();
private:

};


