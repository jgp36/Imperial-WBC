#include <wbc_fri/rt_util.h>

#include "ros/ros.h"
#include <wbc_fri/FriJointState.h>
#include <motion_control_msgs/JointEfforts.h>
#include <wbc_fri/FriJointImpedance.h>
#include <jspace/State.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>
#include <cstdio>

using namespace wbc_fri;
using namespace boost;
using namespace std;
using namespace motion_control_msgs;
using namespace jspace;

static State state(7, 7, 6);
static Matrix A;
static double t;

void stateCallback(const wbc_fri::FriJointState::ConstPtr& msg) {
  t = msg->header.stamp.toSec();
  for (size_t ii(0); ii<7; ++ii) {
	state.position_[ii] = msg->msrJntPos[ii];
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

  //Initial message
  ros::Publisher torque_pub = node.advertise<JointEfforts>("JointEffortCommand", 1000);
  ros::Publisher jointimp_pub = node.advertise<FriJointImpedance>("FriJointImpedance", 1000);
  ros::Subscriber state_sub = node.subscribe("FriJointState", 1000, stateCallback);
  JointEfforts torque_msg;
  FriJointImpedance jointimp_msg;
  FriJointState state_msg;

  for (size_t ii(0); ii<7; ++ii) {
	torque_msg.efforts[ii] = 0;
	jointimp_msg.stiffness[ii] = 0.0;
	jointimp_msg.damping[ii] = 0.0;
  }
  torque_pub.publish(torque_msg);
  jointimp_pub.publish(jointimp_msg);
  
  int count = 0;
  while (ros::ok()) {

    for (size_t ii(0); ii<7; ++ii) {
      torque_msg.efforts[ii] = command[ii];
    }
    torque_pub.publish(torque_msg);

    if (count > 100) {
	cout << "A: " << endl;
	for (size_t ii(0); ii<7; ++ii) {
		for (size_t jj(0); jj<7; ++jj) {
			cout << A(ii,jj) << " " << endl;
		}
		cout << "\n" << endl;
	}
	cout << "Joint Angles: " << endl;
	for (size_t ii(0); ii<7; ++ii) {
		cout << state.position_[ii] << " " << endl;
	}
	cout << "\n" << endl;
    }
    count++;

    ros::spinOnce();
  }
  
  warnx("shutting down");
  ros::shutdown();
}
