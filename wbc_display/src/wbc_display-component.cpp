#include "wbc_display-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <jspace/State.hpp>

Wbc_display::Wbc_display(std::string const& name) : TaskContext(name), rate(1.0), lastTime(0.0){

  this->addPort("RobotState", port_robot_state);
  this->addPort("SkillState", port_skill_state);
  this->addProperty("rate", rate);
}

bool Wbc_display::configureHook(){
  return true;
}

bool Wbc_display::startHook(){
  return true;
}

void Wbc_display::updateHook(){
  if(port_robot_state.read(robot_state) == NewData) {
    if(robot_state.header.stamp.toSec() - lastTime >= 1/rate) {
      port_skill_state.read(skill_state);

      std::cout << skill_state << std::endl;
    
      std::cout << "Joint Position" << std::endl;
      for (size_t ii(0); ii < robot_state.position.size(); ++ii) {
        std:: cout << robot_state.position[ii] << "   ";
      }
      std::cout << std::endl;

      std::cout << "Joint Velocity" << std::endl;
      for (size_t ii(0); ii < robot_state.velocity.size(); ++ii) {
        std:: cout << robot_state.velocity[ii] << "   ";
      }
      std::cout << std::endl;

      std::cout << "Joint Command" << std::endl;
      for (size_t ii(0); ii < robot_state.effort.size(); ++ii) {
        std:: cout << robot_state.effort[ii] << "   ";
      }
      std::cout << std::endl;
  
      lastTime = robot_state.header.stamp.toSec();
    }
  }
}

void Wbc_display::stopHook() {
}

void Wbc_display::cleanupHook() {
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Wbc_display)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Wbc_display)
