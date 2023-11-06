#pragma once
#include "Body.h"
#include "Shape.h"

class Contact;

class Intersections
{
public:
	static bool Intersections::Intersect(Body& a, Body& b, Contact& contact);
};