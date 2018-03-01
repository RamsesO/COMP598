#include <math.h>
#include <stdio.h>
#define M_PI 3.14159265358979323846
#define radius 4.25

// angle you need to turn based on distance traveled and radius
float calculateAngle(float distanceTraveled)
{
	// in radians
	float theta = acos(distanceTraveled/(2*radius));

	// conversion to degrees
	float degrees = theta * 180 / M_PI;

	// need to subtract degrees from 180 to be able to see how much we need to turn
	return 180.0 - degrees;
}

int main(int argc, char** argv)
{

	// lets say the car has already moved across the black circle
	// and found the end. therefore, we now have the distance traveled
	float disT = 7.4;

	float degreesToTurn = calculateAngle(disT);

	// now, is the degrees to turn going to be an addition(move face to right) or subtraction(move face to left)?
	
	// lets assume that it's an addition
	// turn the robot the 'degreesToTurn' (add)
	// move the robot forward the radius amount
	// if found, done!
	// else, it must have been a subtraction
	// move backward the radius amount
	// turn the robot the 'degreesToTurn' TWICE (add)
	// move the robot forward the radius amount OR until it finds the white circle

}