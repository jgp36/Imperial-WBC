#ifndef OROCOS_VIRTUALCAMCOMPONENT_HPP
#define OROCOS_VIRTUALCAMCOMPONENT_HPP

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
#include <std_msgs/typekit/Types.hpp>

using namespace opspace;
using namespace wbc_core_opspace;
using namespace boost;
using namespace std;
using namespace uta_opspace;
using namespace RTT;

namespace wbc_lwr {

  class virtualCamComponent : public RTT::TaskContext{

    //Initialization properties
    string robot_file_;
    string skill_file_;
    bool camera_;
    bool verbose;
    bool init_;

    scoped_ptr<jspace::Model> model;
    shared_ptr<Factory> factory;
    shared_ptr<opspace::ReflectionRegistry> registry;
    shared_ptr<ParamCallbacks> param_cbs;
    shared_ptr<ControllerNG> controller;
    shared_ptr<Skill> skill;
    jspace::State state;
    jspace::Matrix A;
    jspace::Vector gravity;
    jspace::Vector msrJntTrq;
    jspace::Vector estExtJntTrq;

    Vector pos_prev;
    Vector command;
    
    //Camera data and ports
    float period_;
    Vector position_buffer[50];
    
    OutputPort<std_msgs::Float32MultiArray> port_vis_data;
    std_msgs::Float32MultiArray vis_data;

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
    OutputPort<sensor_msgs::JointState> port_robot_state;
    sensor_msgs::JointState robot_state;
    OutputPort<rt_string> port_skill_state;
    rt_string skill_state;
    rt_ostringstream temp_skill_state;
    
    float t_render;
    os::TimeService::ticks t_start;
    os::TimeService::Seconds t_last_out;
    os::TimeService::Seconds t;
    os::TimeService::Seconds t_prev;

  public:
    virtualCamComponent(std::string const& name);
    bool configureHook();
    bool startHook();
    void errorHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void FOAW_best_fit(Vector Yk, Vector *y, size_t len, int n_of_jnt, double d, double T, Vector *y_out);
  };

}
#endif
