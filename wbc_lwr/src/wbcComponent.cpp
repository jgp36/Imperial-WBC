#include "wbcComponent.hpp"
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

wbcComponent::wbcComponent(std::string const& name) 
  : TaskContext(name), init_(false), camera_(false), robot_file_(""), skill_file_(""), verbose(false), t_prev(0){

  //Add ports

  //Inputs -maybe about to switch this to just one of the state inputs
  //JointState
  this->addPort("JointState", port_joint_state);
  //MassMatrix
  //this->addPort("MassMatrix", port_mass_matrix);
  //FriJointState
  //this->addPort("FriJointState", port_fri_joint_state);

  //Outputs
  //JointEffortCommand
  this->addPort("JointEfforts", port_joint_efforts);
  //FriJointImpedance
  //this->addPort("FriJointImpedance", port_fri_joint_impedance);

  //Data Logging Output
  this->addPort("RobotState", port_robot_state);
  this->addPort("SkillState", port_skill_state);

  //Set up parameters
  this->addProperty("robot", robot_file_);
  this->addProperty("skill", skill_file_);
  this->addProperty("camera", camera_);
  this->addProperty("verbose", verbose);

  //Add custom skill additions
  Factory::addSkillType<uta_opspace::RigidTf>("uta_opspace::RigidTf");
  Factory::addSkillType<uta_opspace::SurfaceMotion>("uta_opspace::SurfaceMotion");
  Factory::addSkillType<uta_opspace::SurfaceOriMotion>("uta_opspace::SurfaceOriMotion");
  Factory::addSkillType<uta_opspace::AttachSurface>("uta_opspace::AttachSurface");
  Factory::addSkillType<uta_opspace::KnownAttachSurface>("uta_opspace::KnownAttachSurface");
  Factory::addSkillType<uta_opspace::JointMultiPos>("uta_opspace::JointMultiPos");

}

bool wbcComponent::configureHook(){

  //Robot setup in TAO
  try {
    if (robot_file_.empty()) {
      errx(EXIT_FAILURE, "no robot specification");
    }
    if (verbose) {
      warnx("reading robot spec from %s", robot_file_.c_str());
    }
    static bool const enable_coriolis_centrifugal(false);
    model.reset(jspace::test::parse_sai_xml_file(robot_file_, enable_coriolis_centrifugal));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,
	 "exception while parsing robot specification\n"
	 "  filename: %s\n"
	 "  error: %s",
	 robot_file_.c_str(), ee.what());
  }
  
  factory.reset(new Factory());
  
  //Skill setup
  Status st;
  if (skill_file_.empty()) {
     errx(EXIT_FAILURE, "no skill specification");
  }
  else {
    if (verbose) {
      warnx("reading skills from %s", skill_file_.c_str());
    }
    st = factory->parseFile(skill_file_);
  }
  if ( ! st) {
    errx(EXIT_FAILURE,
	 "failed to parse skills\n"
	 "  specification file: %s\n"
	 "  error description: %s",
	 skill_file_.c_str(), st.errstr.c_str());
  }
  if (verbose) {
    factory->dump(cout, "*** parsed tasks and skills", "* ");
  }


  //Initialization
  state.init(model->getNDOF(), model->getNDOF(), 6);
  command = Vector::Zero(model->getNDOF());
  pos_prev = Vector::Zero(model->getNDOF());

  controller.reset(new ControllerNG("wbc_lwr::servo"));
  param_cbs.reset(new ParamCallbacks());
  // if (verbose) {
  // warnx("initializing param callbacks");
  //}
  registry.reset(factory->createRegistry());
  registry->add(controller);
  //param_cbs->init(node, registry, 1, 100); No ROS node to associate this with...

  //Initilize skill
  if (skill) {
    errx(EXIT_FAILURE,
	 "skill already initialized");
  }
  if (factory->getSkillTable().empty()) {
    errx(EXIT_FAILURE, "empty skill table");
  }
  if ( ! model) {
    errx(EXIT_FAILURE,"no model");
  }
  model->update(state);
  jspace::Status status(controller->init(*model));
  if ( ! status) {
    errx(EXIT_FAILURE,"controller->init() failed: %s", status.errstr.c_str());
  }

  skill = factory->getSkillTable()[0];
  status = skill->init(*model);
  if ( ! status) {
    skill.reset();
    errx(EXIT_FAILURE,"skill->init() failed: %s", status.errstr.c_str());
  }

  //Setup Joint Impedance - always use pure joint impedance - could be setup as a parameter
  /*
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
       fri_joint_impedance.stiffness.push_back(0);
       fri_joint_impedance.damping.push_back(0);
    }
    port_fri_joint_impedance.write(fri_joint_impedance);
  */

  //Initialize port variables
  for (size_t ii(0); ii < model->getNDOF(); ++ii) {
    joint_efforts.efforts.push_back(0);
  }

  //Initialize outputs
  skill_state = "Test string";
  for (size_t ii(0); ii < model->getNDOF(); ++ii) {
    robot_state.position.push_back(state.position_[ii]);
    robot_state.velocity.push_back(state.velocity_[ii]);
    robot_state.effort.push_back(command[ii]);
  }
  port_robot_state.setDataSample(robot_state);

  init_ = true;

  return true;
}

bool wbcComponent::startHook(){
  if (init_) {
  }
  else {
    //this->error(); //This isn't running errorHook for some reason...
    errx(EXIT_FAILURE,"Uninitialized: Run configure first!");
  }
  return true;
}

void wbcComponent::errorHook() {
  std::cout << "wbc_lwr executes errorHook !" <<std::endl;
}

void wbcComponent::updateHook(){

  //Read and update state
  if (port_joint_state.read(joint_state) == NewData) {
    //port_mass_matrix.read(mass_matrix);
    //port_fri_joint_state.read(fri_joint_state);

    double t = joint_state.header.stamp.toSec();
    robot_state.header.stamp.fromSec ( t );
    
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
      state.position_[ii] = joint_state.position[ii];
      robot_state.position[ii] = joint_state.position[ii];
      if (t_prev == 0) {
	state.velocity_[ii] = (state.position_[ii]-pos_prev[ii])/(t-t_prev);
	robot_state.velocity[ii] = state.velocity_[ii];
      }
      else {
	state.velocity_[ii] = 0;
	robot_state.velocity[ii] = 0;
      }
    }
    
    pos_prev = state.position_;
    t_prev = t;
    
    
    //Update model
    model->update(state);
    
    //Compute command
    jspace::Status status(controller->computeCommand(*model, *skill, command));
    if ( ! status) {
      errx(EXIT_FAILURE,"controller->computeCommand() failed: %s", status.errstr.c_str());
    }
    //Command output
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
      joint_efforts.efforts[ii] = command[ii];
      robot_state.effort[ii] = command[ii];
    }
    port_joint_efforts.write(joint_efforts);

    skill->dbg(temp_skill_state, "\n\n**************************************************", "");
    controller->dbg(temp_skill_state, "--------------------------------------------------", "");
    
    skill_state = temp_skill_state.str();
    temp_skill_state.str("");
  }

    //Logger output
    //Write to state
    port_robot_state.write(robot_state);
    port_skill_state.write(skill_state);
}

void wbcComponent::stopHook() {
}

void wbcComponent::cleanupHook() {
  skill.reset();
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
ORO_CREATE_COMPONENT(wbc_lwr::wbcComponent)
