#include "virtualCamComponent.hpp"
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

virtualCamComponent::virtualCamComponent(std::string const& name) 
  : TaskContext(name), init_(false), camera_(false), robot_file_(""), skill_file_(""), verbose(false), t_prev(0), A(jspace::Matrix::Zero(7,7)), gravity(Vector::Zero(7)), msrJntTrq(Vector::Zero(7)), estExtJntTrq(Vector::Zero(7)), t_last_out(0), t_render(0.05) {

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

  //Data Logging Output
  this->addPort("RobotState", port_robot_state);
  this->addPort("SkillState", port_skill_state);
  this->addPort("vis_data", port_vis_data);

  //Set up parameters
  this->addProperty("robot", robot_file_);
  this->addProperty("skill", skill_file_);
  this->addProperty("camera", camera_);
  this->addProperty("verbose", verbose);
   
   
  //Camera properties
  this->addProperty("period", period_);

  //Add custom skill additions
  Factory::addSkillType<uta_opspace::RigidTf>("uta_opspace::RigidTf");
  Factory::addSkillType<uta_opspace::SurfaceMotion>("uta_opspace::SurfaceMotion");
  Factory::addSkillType<uta_opspace::SurfaceOriMotion>("uta_opspace::SurfaceOriMotion");
  Factory::addSkillType<uta_opspace::AttachSurface>("uta_opspace::AttachSurface");
  Factory::addSkillType<uta_opspace::KnownAttachSurface>("uta_opspace::KnownAttachSurface");
  Factory::addSkillType<uta_opspace::JointMultiPos>("uta_opspace::JointMultiPos");
  
  for (size_t ii(0); ii < 50; ++ii) {
    position_buffer[ii] = Vector::Zero(7);
  }

}

bool virtualCamComponent::configureHook(){

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
  state.camData_ = Matrix::Zero(10,3);
  vis_data.data.resize(5+10*3);
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
  
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
       fri_joint_impedance.stiffness[ii] = 0;
       fri_joint_impedance.damping[ii] = 0;
    }
    port_fri_joint_impedance.write(fri_joint_impedance);
  

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

bool virtualCamComponent::startHook(){
  if (init_) {
  }
  else {
    //this->error(); //This isn't running errorHook for some reason...
    errx(EXIT_FAILURE,"Uninitialized: Run configure first!");
  }
  
    t_start = RTT::os::TimeService::Instance()->getTicks();
  return true;
}

void virtualCamComponent::errorHook() {
  std::cout << "wbc_lwr executes errorHook !" <<std::endl;
}

void virtualCamComponent::updateHook(){

    
    t = RTT::os::TimeService::Instance()->getSeconds(t_start);

    Vector c = Vector::Zero(3);
    c(0) = -0.6;
    c(1) =  0.0;
    c(2) =  0.1;
    float R(0.1+0.03*cos(2*M_PI*t/period_));

    for (size_t ii(0); ii < 5; ++ii) {
      state.camData_(ii,0) = c(0);
      state.camData_(ii,1) = c(1) + R*cos(ii*M_PI/4);
      state.camData_(ii,2) = c(2) + R*sin(ii*M_PI/4);
    }
    
    for (size_t ii(0); ii < 5; ++ii) {
      state.camData_(ii+5,0) = c(0)+0.05;
      state.camData_(ii+5,1) = c(1) + R*cos(ii*M_PI/4);
      state.camData_(ii+5,2) = c(2) + R*sin(ii*M_PI/4);
    }

/*    //Virtual camera update
    state.camData_(0,0) = -0.6;
    state.camData_(0,1) = 0.0;
    state.camData_(0,2) = 0.2;
        
    state.camData_(1,0) = -0.6;
    state.camData_(1,1) = 0.05;
    state.camData_(1,2) = 0.15;
    
    state.camData_(2,0) = -0.6;
    state.camData_(2,1) = -0.05;
    state.camData_(2,2) = 0.15;
    
    state.camData_(3,0) = -0.6;
    state.camData_(3,1) = 0.1;
    state.camData_(3,2) = 0.1;
    
    state.camData_(4,0) = -0.6;
    state.camData_(4,1) = -0.1;
    state.camData_(4,2) = 0.1;
    
    state.camData_(5,0) = -0.55;
    state.camData_(5,1) = 0.0;
    state.camData_(5,2) = 0.2;
    
    state.camData_(6,0) = -0.55;
    state.camData_(6,1) = 0.05;
    state.camData_(6,2) = 0.15;
   
    state.camData_(7,0) = -0.55;
    state.camData_(7,1) = -0.05;
    state.camData_(7,2) = 0.15;
    
    state.camData_(8,0) = -0.55;
    state.camData_(8,1) = 0.1;
    state.camData_(8,2) = 0.1;
    
    state.camData_(9,0) = -0.55;
    state.camData_(9,1) = -0.1;
    state.camData_(9,2) = 0.12;*/

  port_fri_joint_impedance.write(fri_joint_impedance);

  //Read and update state
  if (port_joint_state.read(joint_state) == NewData) {
    port_mass_matrix.read(mass_matrix);
    for (size_t ii(0); ii<7; ++ii) {
      for (size_t jj(0); jj<7; ++jj) {
	 A(ii,jj) = mass_matrix.mass[7*ii+jj];
      }
    }

    //Hacks to try to make it function better
    A(5,5) += 0.005;

    controller->setAmatrix(A);
    model->setKukaAMatrix(A);
    //Data collection
    port_fri_joint_state.read(fri_joint_state);
    for (size_t ii(0); ii < 7; ++ii) {
	gravity[ii] = fri_joint_state.gravity[ii];
	msrJntTrq[ii] = fri_joint_state.msrJntTrq[ii];
	estExtJntTrq[ii] = fri_joint_state.estExtJntTrq[ii];
    }
    controller->setgTrq(gravity);
    controller->setMsrJntTrq(msrJntTrq);
    controller->setEstExtJntTrq(estExtJntTrq);
    
    robot_state.header.stamp.fromSec ( t );
    
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
      state.position_[ii] = joint_state.position[ii];
      robot_state.position[ii] = joint_state.position[ii];
   /* if (t_prev == 0) {
	state.velocity_[ii] = (state.position_[ii]-pos_prev[ii])/(t-t_prev);
	robot_state.velocity[ii] = state.velocity_[ii];
      }
      else {
	state.velocity_[ii] = 0;
	robot_state.velocity[ii] = 0;
      }*/
    }
    
    FOAW_best_fit(state.position_, position_buffer, 10, 7, 0.5, 0.005, &state.velocity_);
    
    pos_prev = state.position_;
    t_prev = t;
    
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
      robot_state.velocity[ii] = state.velocity_[ii];
    }
    
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
      robot_state.position[ii] = model->getState().position_[ii];
      robot_state.velocity[ii] = model->getState().velocity_[ii]; 
    }
    port_joint_efforts.write(joint_efforts);
    
    if (t - t_last_out > t_render)   {
    //vis_data R, T, ee_pos, camdata
      vis_data.data[0] = 0.2;
      vis_data.data[1] = 1.5; //XXX Hardcoded garbage
 
    
      jspace::Transform ee_transform;
      model->computeGlobalFrame(model->getNode(6), 0.0, 0.0, 0.2,
			       ee_transform);
      
      vis_data.data[2] = ee_transform(0,3);
      vis_data.data[3] = ee_transform(1,3);
      vis_data.data[4] = ee_transform(2,3);  
    
      for (size_t ii(0); ii < 10; ++ii) {
         for (size_t jj(0); jj < 3; ++jj) {
           vis_data.data[5 + 3*ii+jj] = state.camData_(ii,jj);
         }
      }   
      port_vis_data.write(vis_data);
      t_last_out = t;
    }

  }
/*
    	skill->dbg(cout, "\n\n**************************************************", "");
	//controller->dbg(cout, "--------------------------------------------------", "");
	cout << "--------------------------------------------------\n";
	jspace::pretty_print(model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(controller->getCommand(), cout, "gamma", "  ");
*/

    //Logger output
    port_robot_state.write(robot_state);
    //port_skill_state.write(skill_state);
    
}

void virtualCamComponent::stopHook() {

    //Command output
    for (size_t ii(0); ii < model->getNDOF(); ++ii) {
      joint_efforts.efforts[ii] = 0.0;
      robot_state.effort[ii] = 0.0;
    }
    port_joint_efforts.write(joint_efforts);

}

void virtualCamComponent::cleanupHook() {
  skill.reset();
}

void virtualCamComponent::FOAW_best_fit(Vector Yk, Vector *y, size_t len, int n_of_jnt, double d, double T, Vector *y_out)

{
  int j =0;
  for (j=0; j<n_of_jnt; j++){
    
	int exit = 0;
	int n = 0;
	int i = 2;
	int m;
	double sum1;
	double sum2;
//	bool not_done;
//	bool not_done2;
	double a;
	double b;
	double Yl;
	int k = len-1;

  
  //new version************************************************************************
 
      for ( i=1; i< k+1 ; i++){
	    y[i-1](j) = y[i](j);  
	}
	
      y[k](j) = Yk(j); //*q?
	
      do{
	   n = n + 1;
	   a = ( k*y[k-n](j) + (n-k)*(y[k](j)) )/n;
	   sum1=0;
	    sum2=0;
	for (m =0; m <= n; m++){
	   	sum1 = sum1 + y[k-m](j);
	   	sum2 = sum2 + m * y[k-m](j);
	   }
	   
	   b = ((n*sum1 - 2 * sum2)/((n+1)*(n+2)));
	   b = 6*b/(n*T);

//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   i = 0;
	   do{
		i = i + 1;
		Yl = a + (b*(k-i)*T);
	   } while((fabs((double)(y[k-i](j))-Yl)) <= d && i<n);
	    
	   if (i != n || n == k)
		exit = 1;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      } while(exit == 0);
	  
//n=n-1;
    if(n<=2)
	n=2;


    if (n == k) 
	(*y_out)(j) = ( y[k](j) - y[k-n+1](j) )/((n-1)*T); 
    else
	(*y_out)(j) = ( y[k](j) - y[k-n](j) )/(n*T); 
  }///////////////////////////////////////////////////////
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
ORO_CREATE_COMPONENT(wbc_lwr::virtualCamComponent)
