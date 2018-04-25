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
	int elbow_dof = 4;

	// trajectory
	double max_x = 0.4;
	double min_x = -0.4;
	double theta = 0.;
	double theta_min = -M_PI / 3.;
	double theta_max = M_PI / 3.;
	double center_z = 0.5;
	double move_step = 1.0 / 10000;
	int direction = 1; // 1 is positive, -1 is negative

	Vector3d position = Vector3d(0.0, 0.0, 0.0);
	Vector3d elbow_position = Vector3d(0.0, 0.0, 0.0);
	Matrix3d rotation = Matrix3d::Zero();
	Vector3d initial_position = Vector3d::Zero();
	Vector3d desired_position = Vector3d(0.0, -0.5, center_z);
	Vector3d desired_initial_position = Vector3d(desired_position);
	double elbow_z_des = 0.3;
	Matrix3d desired_orientation = Matrix3d::Zero();
	desired_orientation << 0., 1., 0.,
							1., 0., 0.,
							0., 0., -1.;
	// Quaterniond desired_orientation_quat = Quaterniond(0., 1., 0., 0.);
	Vector3d linear_velocity = Vector3d(0.0, 0.0, 0.0);
	Vector3d elbow_linear_velocity = Vector3d(0.0, 0.0, 0.0);
	Vector3d angular_velocity = Vector3d(0.0, 0.0, 0.0);

	Vector3d position_error = Vector3d(0.0, 0.0, 0.0);
	Vector3d elbow_error = Vector3d(0.0, 0.0, 0.0);
	double elbow_z_error = 0.;
	Vector3d rotation_error = Vector3d(0.0, 0.0, 0.0);
	VectorXd pos_rot_error = VectorXd::Zero(6);
	VectorXd velocity = VectorXd::Zero(6);
	VectorXd force_1 = VectorXd::Zero(3);
	VectorXd force_2 = VectorXd::Zero(3);
	VectorXd force_3 = VectorXd::Zero(1);
	VectorXd dq = VectorXd::Zero(dof);
	VectorXd gamma_diss = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	VectorXd command_torques = VectorXd::Zero(dof);

	MatrixXd mass;
	MatrixXd mass_inv;
	MatrixXd Jac_v = MatrixXd::Zero(3, dof);
	MatrixXd Jac_elbow_v = MatrixXd::Zero(3, elbow_dof);
	MatrixXd Jac_w = MatrixXd::Zero(3, dof);
	MatrixXd Jac_1 = MatrixXd::Zero(3, dof);
	MatrixXd Jac_2 = MatrixXd::Zero(3, dof); // projected jacobian for rotational task
	MatrixXd Jac_3 = MatrixXd::Zero(1, dof); // projected jacobian for elbow task
	MatrixXd J_bar_1 = MatrixXd::Zero(dof, 3);
	MatrixXd J_bar_2 = MatrixXd::Zero(dof, 3);
	MatrixXd J_bar_3 = MatrixXd::Zero(dof, 1);
	MatrixXd N1;
	MatrixXd N2; // null space matrix for rotational task
	MatrixXd N3; // null space matrix for elbow task
	MatrixXd lambda_1 = MatrixXd::Zero(3, 3);
	MatrixXd lambda_2 = MatrixXd::Zero(3, 3);
	MatrixXd lambda_3 = MatrixXd::Zero(1, 1);

	const MatrixXd I7 = MatrixXd::Identity(dof, dof);

	string ee_link = "link7";
	string elbow_link = "link4";
	Vector3d ee_pos = Vector3d(0.0, 0.0, 0.025);
	Vector3d elbow_pos = Vector3d(0.0, 0.0, 0.0);

	double kp = 100.0;
	double kv = 30.0;
	double kvq = 20.0;

	robot->position(position, ee_link, ee_pos);
	robot->position(initial_position, ee_link, ee_pos);
	robot->rotation(rotation, ee_link);
	
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
		if (controller_counter < control_freq * 5) {
			// in the initial 5 seconds, gradually move position to desired position
			Vector3d position_step = (desired_initial_position - initial_position) / (control_freq * 5);
			desired_position = initial_position + position_step * controller_counter;
		} else {
			// start controlling with sine trajectory
			auto x = desired_position[0];
			if (x > max_x) {
				direction = -1;
			} else if (x < min_x) {
				direction = 1;
			}
			desired_position[0] += direction * move_step;
			x = desired_position[0];
			desired_position[2] = center_z + 0.1 * sin(x * 20.);
		}

		// update the model 20 times slower (hacky, should use a separate thread)
		if(controller_counter%20 == 0)
		{
			robot->updateModel();
			mass = robot->_M;
			mass_inv = robot->_M_inv;
			robot->coriolisForce(coriolis);
			robot->Jv(Jac_v, ee_link, ee_pos);
			robot->Jw(Jac_w, ee_link);
			robot->Jv(Jac_elbow_v, elbow_link, elbow_pos);
			
			// task 1: position control
			Jac_1 = Jac_v;
			lambda_1 = (Jac_1 * mass_inv * Jac_1.transpose()).inverse();
			J_bar_1 = mass_inv * Jac_1.transpose() * lambda_1;
			N1 = (I7 - J_bar_1 * Jac_1);

			// task 2: orientation control
			Jac_2 = Jac_w * N1;
			lambda_2 = (Jac_2 * mass_inv * Jac_2.transpose()).inverse();
			J_bar_2 = mass_inv * Jac_2.transpose() * lambda_2;
			N2 = (I7 - J_bar_2 * Jac_2) * N1;

			// task 3: keep elbow flat
			Jac_3 = (Jac_elbow_v * N2).block<1,7>(2, 0);
			lambda_3 = (Jac_3 * mass_inv * Jac_3.transpose()).inverse();
			// cout << "lambda_3: " << lambda_3 << endl;
			J_bar_3 = mass_inv * Jac_3.transpose() * lambda_3;
			N3 = (I7 - J_bar_3 * Jac_3) * N2;
		}

		dq = robot->_dq;
		robot->position(position, ee_link, ee_pos);
		robot->position(elbow_position, elbow_link, elbow_pos);
		robot->rotation(rotation, ee_link);
		robot->linearVelocity(linear_velocity, ee_link, ee_pos);
		robot->linearVelocity(elbow_linear_velocity, elbow_link, elbow_pos);
		robot->angularVelocity(angular_velocity, ee_link);

		// Compute joint torques
		double time = controller_counter / control_freq;

		// position control
		position_error = position - desired_position;
		force_1 = lambda_1 * (-kp * position_error - kv * linear_velocity);
		
		// rotation control
		Sai2Model::orientationError(rotation_error, desired_orientation, rotation);
		// angular_velocity = Jac_2 * dq;
		force_2 = lambda_2 * (-kp * rotation_error - kv * angular_velocity);
		
		// elbow control
		// elbow_z_error = elbow_position(2) - elbow_z_des;
		// cout << "elbow_z_error: " << elbow_z_error << endl;
		// elbow_error << 0., 0., elbow_z_error;
		// elbow_linear_velocity << 0., 0., elbow_linear_velocity(2);
		// cout << "elbow_linear_velocity: " << elbow_linear_velocity << endl;
		// force_3 << lambda_3(0) * (-kp * elbow_z_error - kv * elbow_linear_velocity(2));

		// joint damping
		gamma_diss = -kvq * mass * dq;

		command_torques = Jac_1.transpose() * force_1
							+ N1.transpose() * Jac_2.transpose() * force_2
							// + N2.transpose() * Jac_3.transpose() * force_3;
							+ N2.transpose() * gamma_diss;
		
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
