#include "Scene.h"
#include "../Contact.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"
#include "../Utils.h"

Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

void Scene::Initialize() 
{
	float incrementalAngle = 0;
	float radiusArena = 5;
	float gap = 6;
	float n_balls = 20;

	Body ball_arena;
	for (int i = 0; i < n_balls; i++)
	{
		ball_arena.position = Vec3(cos(incrementalAngle) * radiusArena * gap, sin(incrementalAngle) * radiusArena * gap, 0);
		ball_arena.orientation = Quat(0, 0, 0, 1);
		ball_arena.shape = new ShapeSphere(radiusArena);
		ball_arena.inverseMass = 0.00f;
		ball_arena.elasticity = 0.5f;
		ball_arena.friction = 0.05f;
		ball_arena.linearVelocity = Vec3(0, 0, 0);
		incrementalAngle += 2 * 3.14159265 / n_balls;
		bodies.push_back(ball_arena);
	}

	Body ball_ground;
	for (size_t i = 0; i < 60; i++)
	{
		std::pair<double, double> randomCoord = Utils::getRandomCoordinate(radiusArena * gap);
		ball_ground.position = Vec3(randomCoord.first, randomCoord.second, -4);
		ball_ground.orientation = Quat(0, 0, 0, 1);
		ball_ground.shape = new ShapeSphere(radiusArena);
		ball_ground.inverseMass = 0.00f;
		ball_ground.elasticity = 0.5f;
		ball_ground.friction = 0.05f;
		ball_ground.linearVelocity = Vec3(0, 0, 0);
		bodies.push_back(ball_ground);
	}

	float radius = 10000.0f;

	Body earth;
	earth.position = Vec3(0, 0, -radius);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(radius);
	earth.inverseMass = 0.0f;
	earth.elasticity = 0.5f;
	earth.friction = 0.5f;
	bodies.push_back(earth);

	Body cochonnet;
	cochonnet.position = Vec3(0, 0, 100);
	cochonnet.orientation = Quat(0, 0, 0, 1);
	cochonnet.shape = new ShapeSphere(.7f);
	cochonnet.inverseMass = 0.0f;
	cochonnet.elasticity = 0.1f;
	cochonnet.friction = 0.5f;
	bodies.push_back(cochonnet);
	cochonnetIndex = bodies.size() - 1;

	std::cout << "press f to launch the cochonnet" << std::endl;
}

void Scene::Update(const float dt_sec)
{
	// Gravity
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);
	}
	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);
	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)_malloca(sizeof(Contact) * maxContacts);

	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}
	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact),
			Contact::CompareContact);
	}
	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;
		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}
	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}
}

void Scene::OnKeyPress(const char* key)
{
	if (key == "F")
	{
		std::cout << "f key pressed" << std::endl;
		float radius = 0.5f;
		float x = 0;
		float y = 0;
		float z = 10;

		camRot = Vec3(camPos.x * -1, camPos.y * -1, camPos.z * -1);

		bodies[cochonnetIndex].position = (camPos);
		bodies[cochonnetIndex].inverseMass = 0.9f;
		bodies[cochonnetIndex].angularVelocity.Zero();
		bodies[cochonnetIndex].linearVelocity = camRot * 10;
	}
}
