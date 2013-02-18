#ifndef OROCOS_WBC_DISPLAY_COMPONENT_HPP
#define OROCOS_WBC_DISPLAY_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.hpp>
#include <rtt/rt_string.hpp>

using namespace RTT;

class Wbc_display : public RTT::TaskContext{

  InputPort<sensor_msgs::JointState> port_robot_state;
  sensor_msgs::JointState robot_state;
  InputPort<rt_string> port_skill_state;
  rt_string skill_state;

  double rate;
  double lastTime;
  
  public:
    Wbc_display(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
