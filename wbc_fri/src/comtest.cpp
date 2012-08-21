#include <wbc_fri/rt_util.h>

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <motion_control_msgs/JointEfforts.h>
#include <wbc_fri/FriJointImpedance.h>
#include <wbc_fri/MassMatrix.h>
#include <jspace/State.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>
#include <cstdio>

using namespace wbc_fri;
using namespace boost;
using namespace std;
using namespace motion_control_msgs;
using namespace sensor_msgs;
using namespace jspace;

static State state(7, 7, 6);
static Matrix A;
static double t;

void stateCallback(const JointState::ConstPtr& msg) {
  t = msg->header.stamp.toSec();
  for (size_t ii(0); ii<7; ++ii) {
	state.position_[ii] = msg->position[ii];
  }
}

void massCallback(const MassMatrix::ConstPtr& msg) {
  for (size_t ii(0); ii<7; ++ii) {
	for (size_t jj(0); jj<7; ++jj) {
		A(ii,jj) = msg->mass[7*ii+jj];
	}
  }
}



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "wbc_fri_comtest", ros::init_options::NoSigintHandler);
  ros::NodeHandle node("~");

  Vector command(Vector::Zero(7));
  A = Matrix::Zero(7,7);
  t = 0;
  double kp = 2.0;

  //Initial message
  ros::Publisher torque_pub = node.advertise<JointEfforts>("/JointEffortCommand", 1);
  ros::Publisher jointimp_pub = node.advertise<FriJointImpedance>("/FriJointImpedance", 1);
  ros::Subscriber state_sub = node.subscribe("/JointState", 1, stateCallback);
  ros::Subscriber mass_sub = node.subscribe("/MassMatrix", 1000, massCallback);
  JointEfforts torque_msg;
  FriJointImpedance jointimp_msg;

  for (size_t ii(0); ii<7; ++ii) {
	torque_msg.efforts.push_back(0.0);
	jointimp_msg.stiffness[ii] = 0.0;
	jointimp_msg.damping[ii] = 1.0;
  }
  torque_pub.publish(torque_msg);
  jointimp_pub.publish(jointimp_msg);
  
  double begin = ros::Time::now().toSec();
  while (ros::ok()) {

    for (size_t ii(0); ii<7; ++ii) {
      //torque_msg.efforts[ii] = -kp*state.position_[ii];
    }
    jointimp_pub.publish(jointimp_msg);
    torque_pub.publish(torque_msg);

    if (ros::Time::now().toSec() - begin > 1.0) {
	begin = ros::Time::now().toSec();
	jointimp_pub.publish(jointimp_msg);
	cout << "A: " << endl;
	for (size_t ii(0); ii<7; ++ii) {
		for (size_t jj(0); jj<7; ++jj) {
			cout << A(ii,jj) << "  ";
		}
		cout << "\n";
	}
	cout << "\n";
	cout << "Joint Angles at time " << t << ": " << endl;
	for (size_t ii(0); ii<7; ++ii) {
		cout << state.position_[ii] << " ";
	}
	cout << "\n" << endl;
	cout << "Torque desired: " << endl;
	for (size_t ii(0); ii<7; ++ii) {
		cout << -kp*state.position_[ii] << " ";
	}
	cout << "\n" << endl;
    }

    ros::spinOnce();
  }
  
  warnx("shutting down");
  ros::shutdown();
}
