#pragma once
#include <vector>

#include "../Body.h"

class Scene 
{
public:
	Scene() { bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	
	void OnKeyPress(const char* key);

	std::vector<Body> bodies;
	Body earth;

	bool isCochonnetPlayed = false;
	Body cochonnet;
	std::vector<Body> petanque_balls;
};

