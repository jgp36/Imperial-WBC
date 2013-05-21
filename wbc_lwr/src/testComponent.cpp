#include "testComponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <uta_opspace/RigidTf.hpp>
#include <uta_opspace/SurfaceMotion.hpp>
#include <uta_opspace/SurfaceOriMotion.hpp>
#include <uta_opspace/AttachSurface.hpp>
#include <uta_opspace/KnownAttachSurface.hpp>
#include <uta_opspace/JointMultiPos.hpp>

using namespace opspace;
using namespace boost;
using namespace std;
using namespace uta_opspace;

namespace wbc_lwr {

testComponent::testComponent(std::string const& name) 
  : TaskContext(name), init_(false), camera_(false), robot_file_(""), skill_file_(""), verbose(false), t_prev(0), t_last(0), t_out(1){

  //Add ports

  //Inputs -maybe about to switch this to just one of the state inputs
  //JointState
  this->addPort("JointState", port_joint_state);
  //MassMatrix
  this->addPort("MassMatrix", port_mass_matrix);
  //FriJointState
  this->addPort("FriJointState", port_fri_joint_state);

  //Outputs
  //JointEffortCommand
  this->addPort("JointEfforts", port_joint_efforts);
  //FriJointImpedance
  this->addPort("FriJointImpedance", port_fri_joint_impedance);


}

bool testComponent::configureHook(){


  //Setup Joint Impedance - always use pure joint impedance - could be setup as a parameter
    for (size_t ii(0); ii < 7; ++ii) {
       fri_joint_impedance.stiffness[ii] = 0;
       fri_joint_impedance.damping[ii] = 0;
    }
    //port_fri_joint_impedance.write(fri_joint_impedance);

  //Initialize port variables
  for (size_t ii(0); ii < 7; ++ii) {
    joint_efforts.efforts.push_back(0);
  }

   port_joint_efforts.write(joint_efforts);

  init_ = true;

  return true;
}

bool testComponent::startHook(){
  if (init_) {
  }
  else {
    //this->error(); //This isn't running errorHook for some reason...
    errx(EXIT_FAILURE,"Uninitialized: Run configure first!");
  }
  return true;
}

void testComponent::errorHook() {
  std::cout << "wbc_lwr executes errorHook !" <<std::endl;
}

void testComponent::updateHook(){

  //Read and update state
  if (port_joint_state.read(joint_state) == NewData) {
    port_mass_matrix.read(mass_matrix);
    port_fri_joint_state.read(fri_joint_state);
    port_joint_state.read(joint_state);

    double t = joint_state.header.stamp.toSec();
    
    for (size_t ii(0); ii < 7; ++ii) {
      state.position_[ii] = joint_state.position[ii];
      if (t_prev == 0) {
	state.velocity_[ii] = (state.position_[ii]-pos_prev[ii])/(t-t_prev);
      }
      else {
	state.velocity_[ii] = 0;
      }
    }
    
    pos_prev = state.position_;
    t_prev = t;

    if (t - t_last > t_out) {
	jspace::pretty_print(state.position_, cout, "jpos", "  ");
        jspace::pretty_print(state.velocity_, cout, "jvel", "  ");
	t_last = t;
    }
    
   }

   port_joint_efforts.write(joint_efforts);
}

void testComponent::stopHook() {
}

void testComponent::cleanupHook() {
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(wbc_lwr)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
}
ORO_CREATE_COMPONENT(wbc_lwr::testComponent)
