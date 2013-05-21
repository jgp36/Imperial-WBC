#ifndef OROCOS_TESTCOMPONENT_HPP
#define OROCOS_TESTCOMPONENT_HPP

#include <rtt/RTT.hpp>

#include <jspace/State.hpp>
#include <jspace/test/sai_util.hpp>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <uta_opspace/ControllerNG.hpp>
#include <wbc_core/opspace_param_callbacks.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>

#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <rtt/rt_string.hpp>

using namespace opspace;
using namespace wbc_core_opspace;
using namespace boost;
using namespace std;
using namespace uta_opspace;
using namespace RTT;

namespace wbc_lwr {

  class testComponent : public RTT::TaskContext{

    //Initialization properties
    string robot_file_;
    string skill_file_;
    bool camera_;
    bool verbose;
    bool init_;

    jspace::State state;

    Vector pos_prev;
    double t_prev;
    Vector command;

    double t_last;
    double t_out;

    //Ports and messages
    InputPort<sensor_msgs::JointState> port_joint_state;
    sensor_msgs::JointState joint_state;
    InputPort<lwr_fri::MassMatrix> port_mass_matrix;
    lwr_fri::MassMatrix mass_matrix;
    InputPort<lwr_fri::FriJointState> port_fri_joint_state;
    lwr_fri::FriJointState fri_joint_state;
    OutputPort<motion_control_msgs::JointEfforts> port_joint_efforts;
    motion_control_msgs::JointEfforts joint_efforts;
    OutputPort<lwr_fri::FriJointImpedance> port_fri_joint_impedance;
    lwr_fri::FriJointImpedance fri_joint_impedance;

  public:
    testComponent(std::string const& name);
    bool configureHook();
    bool startHook();
    void errorHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
  };

}
#endif
