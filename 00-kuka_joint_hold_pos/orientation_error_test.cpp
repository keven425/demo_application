#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <tinyxml2.h>

#include <signal.h>
#include <math.h>

using namespace std;
using namespace Eigen;

int main() {
	Vector3d rotation_error = Vector3d(0.0, 0.0, 0.0);

	Matrix3d desired_orientation;
	desired_orientation << 1., 0., 0.,
							0., 1., 0.,
							0., 0., -1.;
	Matrix3d current_orientation;
	current_orientation << 0.633458, -0.673123,  0.381624,
 							-0.67511, -0.239795,  0.697657,
							-0.378098, -0.699574, -0.606331;
	Sai2Model::orientationError(rotation_error, desired_orientation, current_orientation);
	cout << "rotation matrix rotation_error: " << rotation_error << endl;


	Quaterniond desired_orientation_q = Quaterniond(1., 0., 0., 0.);
	Quaterniond current_orientation_q = Quaterniond(-0.4436585, 0.7873348, -0.4281003, 0.0011196);
	Sai2Model::orientationError(rotation_error, desired_orientation_q, current_orientation_q);
	cout << "quaternion rotation_error: " << rotation_error << endl;

	/*
		rotation matrix rotation_error: -0.0009585
		 -0.001763
		-0.0009935
		quaternion rotation_error:   -1.57467
		  0.856201
		-0.0022392
	*/
}