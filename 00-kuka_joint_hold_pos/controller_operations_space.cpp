// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <tinyxml2.h>

#include <signal.h>
#include <math.h>

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/00-kuka_joint_hold_pos/iiwa7.urdf";
const std::string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

const bool simulation = true;
// const bool simulation = false;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;

int main() {
	if(simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force_moment";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";
	}

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the joint controller        /////
	////////////////////////////////////////////////

	robot->updateModel();

	int dof = robot->dof();
	Vector3d position = Vector3d(0.0, 0.0, 0.0);
	Vector3d desired_position = Vector3d(0.0, 0.5, 0.2);
	Vector3d velocity = Vector3d(0.0, 0.0, 0.0);
	Vector3d force = Vector3d(0.0, 0.0, 0.0);
	VectorXd dq = VectorXd::Zero(dof);
	VectorXd gamma_diss = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd mass;
	MatrixXd mass_inv;
	MatrixXd Jac_v;
	MatrixXd J_bar;
	MatrixXd N;
	MatrixXd lambda;

	const MatrixXd I7 = MatrixXd::Identity(dof, dof);

	string ee_link = "link7";
	Vector3d ee_pos = Vector3d(0.0, 0.0, 0.025);

	double kp = 100.0;
	double kv = 30.0;
	double kvq = 20.0;

	robot->position(position, "link7", Vector3d(0.0, 0.0, 0.025));
	double max_x = 0.6;
	double min_x = -0.6;
	double center_y = 0.3;
	double move_step = 1.0 / 10000;
	int direction = 1; // 1 is positive, -1 is negative

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		// set desired end effector position
		auto x = desired_position[0];
		if (x > max_x) {
			direction = -1;

		} else if (x < min_x) {
			direction = 1;
		}
		desired_position[0] += direction * move_step;
		x = desired_position[0];
		desired_position[2] = center_y + 0.3 * sin(x * 20.);
		
		// update the model 20 times slower (hacky, should use a separate thread)
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			mass = robot->_M;
			mass_inv = robot->_M_inv;
			robot->coriolisForce(coriolis);
			robot->Jv(Jac_v, ee_link, ee_pos);
			lambda = (Jac_v * mass_inv * Jac_v.transpose()).inverse();
			J_bar = mass_inv * Jac_v.transpose() * lambda;
			N = (I7 - J_bar * Jac_v);
		}

		// Compute joint torques
		double time = controller_counter/control_freq;

		dq = robot->_dq;
		robot->position(position, ee_link, ee_pos);
		robot->linearVelocity(velocity, ee_link, ee_pos);
		force = lambda * (-kp * (position - desired_position) - kv * velocity);
		gamma_diss = -kvq * mass * dq;
		command_torques = Jac_v.transpose() * force + N.transpose() * gamma_diss + coriolis;
		
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
