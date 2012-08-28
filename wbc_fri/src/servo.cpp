#include <wbc_fri/rt_util.h>

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <motion_control_msgs/JointEfforts.h>
#include <wbc_fri/FriJointImpedance.h>
#include <wbc_fri/MassMatrix.h>
#include <wbc_fri/FriJointState.h>
#include <jspace/test/sai_util.hpp>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <uta_opspace/ControllerNG.hpp>
#include <wbc_core/opspace_param_callbacks.hpp>
#include <uta_opspace/RigidTf.hpp>
#include <uta_opspace/SurfaceMotion.hpp>
#include <uta_opspace/SurfaceOriMotion.hpp>
#include <uta_opspace/AttachSurface.hpp>
#include <uta_opspace/KnownAttachSurface.hpp>
#include <uta_opspace/JointMultiPos.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>

#include <wbc_fri/cs8c_interface.h>
#include <wbc_fri/udp_osi.h>
#include <wbc_fri/ndtypes.h>

using namespace wbc_fri;
using namespace opspace;
using namespace wbc_core_opspace;
using namespace uta_opspace;
using namespace boost;
using namespace std;
using namespace motion_control_msgs;
using namespace sensor_msgs;

static char const * opspace_fallback_str = 
  "- tasks:\n"
  "  - type: opspace::PositionTask\n"
  "    name: eepos\n"
  "    end_effector_id: 6\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  10.0 ]\n"
  "    maxvel: [ 0.5 ]\n"
  "    maxacc: [ 1.5 ]\n"
  "  - type: opspace::PostureTask\n"
  "    name: posture\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  10.0 ]\n"
  "    maxvel: [ 3.1416 ]\n"
  "    maxacc: [ 6.2832 ]\n"
  "- skills:\n"
  "  - type: opspace::TPSkill\n"
  "    name: task_posture\n"
  "    default:\n"
  "      eepos: eepos\n"
  "      posture: posture\n";

static bool verbose(false);
static bool cam(false);
static scoped_ptr<jspace::Model> model;
static shared_ptr<Factory> factory;
static shared_ptr<opspace::ReflectionRegistry> registry;
static shared_ptr<ParamCallbacks> param_cbs;
static shared_ptr<ControllerNG> controller;
static jspace::State state(7, 7, 6);
//static std::vector<vel_est*> vest;
static Vector pos_prev;
static double t_prev;

void stateCallback(const JointState::ConstPtr& msg) {
  //update State
  double t = msg->header.stamp.toSec();
  for (size_t ii(0); ii<7; ++ii) {
	state.position_[ii] = msg->position[ii];
	/*double pos = state.position_[ii];
	if (vest.size() != 7) {
	  vest.push_back(new vel_est());
	  vest[ii]->init(2.6e-7,0.02,100,pos,t);
	}
	state.velocity_[ii] = vest[ii]->update(pos,t);*/
	if (t_prev != 0) {
		state.velocity_[ii] = (state.position_[ii]-pos_prev[ii])/(t-t_prev);
	}
	else {
		state.velocity_[ii] = 0.0;
	}
  }
   pos_prev = state.position_;
   t_prev = t;
}

void massCallback(const MassMatrix::ConstPtr& msg) {
  jspace::Matrix A(jspace::Matrix::Zero(7,7));
  for (size_t ii(0); ii<7; ++ii) {
	for (size_t jj(0); jj<7; ++jj) {
		A(ii,jj) = msg->mass[7*ii+jj];
	}
  }
  controller->setAmatrix(A);
}

//Pure data collection
void friCallback(const FriJointState::ConstPtr& msg) {
	Vector gravity(Vector::Zero(7));
	Vector msrJntTrq(Vector::Zero(7));
	Vector estExtJntTrq(Vector::Zero(7));
	for (size_t ii(0); ii < 7; ++ii) {
		gravity[ii] = msg->gravity[ii];
		msrJntTrq[ii] = msg->msrJntTrq[ii];
		estExtJntTrq[ii] = msg->estExtJntTrq[ii];
	}
	controller->setgTrq(gravity);
	controller->setMsrJntTrq(msrJntTrq);
	controller->setEstExtJntTrq(estExtJntTrq);
}



static void usage(int ecode, std::string msg)
{
  errx(ecode,
       "%s\n"
       "  options:\n"
       "  -h               help (this message)\n"
       "  -v               verbose mode\n"
       "  -c               camera mode\n"
       "  -r  <filename>   robot specification (SAI XML format)\n"
       "  -s  <filename>   skill specification (YAML file with tasks etc)",
       msg.c_str());
}


static void parse_options(int argc, char ** argv)
{
  string skill_spec("");
  string robot_spec("");
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      usage(EXIT_FAILURE, "problem with option `" + string(argv[ii]) + "'");
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(EXIT_SUCCESS, "servo [-h] [-v] [-s skillspec] -r robotspec");
	break;

      case 'c':
	cam = true;
        break;

      case 'v':
	verbose = true;
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-r requires parameter");
 	}
	robot_spec = argv[ii];
 	break;
	
      case 's':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-s requires parameter");
 	}
	skill_spec = argv[ii];
 	break;
	
      default:
	usage(EXIT_FAILURE, "invalid option `" + string(argv[ii]) + "'");
      }
  }
  
  try {
    if (robot_spec.empty()) {
      usage(EXIT_FAILURE, "no robot specification (see option -r)");
    }
    if (verbose) {
      warnx("reading robot spec from %s", robot_spec.c_str());
    }
    static bool const enable_coriolis_centrifugal(false);
    model.reset(jspace::test::parse_sai_xml_file(robot_spec, enable_coriolis_centrifugal));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,
	 "exception while parsing robot specification\n"
	 "  filename: %s\n"
	 "  error: %s",
	 robot_spec.c_str(), ee.what());
  }
  
  factory.reset(new Factory());
  
  Status st;
  if (skill_spec.empty()) {
    if (verbose) {
      warnx("using fallback task/posture skill");
    }
    st = factory->parseString(opspace_fallback_str);
  }
  else {
    if (verbose) {
      warnx("reading skills from %s", skill_spec.c_str());
    }
    st = factory->parseFile(skill_spec);
  }
  if ( ! st) {
    errx(EXIT_FAILURE,
	 "failed to parse skills\n"
	 "  specification file: %s\n"
	 "  error description: %s",
	 skill_spec.c_str(), st.errstr.c_str());
  }
  if (verbose) {
    factory->dump(cout, "*** parsed tasks and skills", "* ");
  }
}


static void handle(int signum)
{
  if (ros::ok()) {
    warnx("caught signal, requesting shutdown");
    ros::shutdown();
  }
  else {
    errx(EXIT_SUCCESS, "caught signal (again?), attempting forced exit");
  }
}

namespace {
  
  class Servo
    : public RTUtil
  {
  public:
    shared_ptr<Skill> skill;    
    
    virtual int init(jspace::State const & state) {
      if (skill) {
	warnx("Servo::init(): already initialized");
	return -1;
      }
      if (factory->getSkillTable().empty()) {
	warnx("Servo::init(): empty skill table");
	return -2;
      }
      if ( ! model) {
	warnx("Servo::init(): no model");
	return -3;
      }
      
      model->update(state);
    
      jspace::Status status(controller->init(*model));
      if ( ! status) {
	warnx("Servo::init(): controller->init() failed: %s", status.errstr.c_str());
	return -4;
      }
      
      skill = factory->getSkillTable()[0]; // XXXX to do: allow selection at runtime
      status = skill->init(*model);
      if ( ! status) {
	warnx("Servo::init(): skill->init() failed: %s", status.errstr.c_str());
	skill.reset();
	return -5;
      }
      
      return 0;
    }
    
    
    virtual int update(jspace::State const & state,
		       jspace::Vector & command)
    {
      if ( ! skill) {
	warnx("Servo::update(): not initialized\n");
	return -1;
      }

      model->update(state);

      jspace::Status status(controller->computeCommand(*model, *skill, command));
      if ( ! status) {
	warnx("Servo::update(): controller->computeCommand() failed: %s", status.errstr.c_str());
	return -2;
      }
      return 0;
    }
    
    
    virtual int cleanup(void)
    {
      skill.reset();
      return 0;
    }
    
  };
  
}

int main(int argc, char ** argv)
{
  struct sigaction sa;
  bzero(&sa, sizeof(sa));
  sa.sa_handler = handle;
  if (0 != sigaction(SIGINT, &sa, 0)) {
    err(EXIT_FAILURE, "sigaction");
  }
  
  // Before we attempt to read any tasks and skills from the YAML
  // file, we need to inform the static type registry about custom
  // additions such as the HelloGoodbyeSkill.
  Factory::addSkillType<uta_opspace::RigidTf>("uta_opspace::RigidTf");
  Factory::addSkillType<uta_opspace::SurfaceMotion>("uta_opspace::SurfaceMotion");
  Factory::addSkillType<uta_opspace::SurfaceOriMotion>("uta_opspace::SurfaceOriMotion");
  Factory::addSkillType<uta_opspace::AttachSurface>("uta_opspace::AttachSurface");
  Factory::addSkillType<uta_opspace::KnownAttachSurface>("uta_opspace::KnownAttachSurface");
  Factory::addSkillType<uta_opspace::JointMultiPos>("uta_opspace::JointMultiPos");
  
  
  ros::init(argc, argv, "wbc_fri_servo", ros::init_options::NoSigintHandler);
  parse_options(argc, argv);
  ros::NodeHandle node("~");

  jspace::Vector command(Vector::Zero(7));
  pos_prev = Vector::Zero(7);
  //Need to setup the transform here
  //Should be changed to a parameter in TAO
  jspace::Matrix R(Matrix::Identity(3,3));
  R(0,0) = 0.9221;
  R(0,1) = -0.0576;
  R(0,2) = -0.3827;
  R(1,0) = 0.3791;
  R(1,1) = -0.0643;
  R(1,2) = 0.9231;
  R(2,0) = -0.0778;
  R(2,1) = -0.9963;
  R(2,2) = -0.0374;
  jspace::Matrix d(Matrix::Zero(3,1));
  /*d(0,0) = -1.0877;
  d(1,0) = 2.8646;
  d(2,0) = 0.0528;*/
  d(0,0) = -1.0877;
  d(1,0) = 3.0146;
  d(2,0) = 0.0428;
  
  controller.reset(new ControllerNG("wbc_fri::servo"));
  param_cbs.reset(new ParamCallbacks());
  Servo servo;
  if (verbose) {
    warnx("initializing param callbacks");
  }
  registry.reset(factory->createRegistry());
  registry->add(controller);
  param_cbs->init(node, registry, 1, 100);

  //Initial message
  ros::Publisher torque_pub = node.advertise<JointEfforts>("/JointEffortCommand", 1000);
  ros::Publisher jointimp_pub = node.advertise<FriJointImpedance>("/FriJointImpedance", 1000);
  ros::Subscriber state_sub = node.subscribe("/JointState", 1000, stateCallback);
  ros::Subscriber mass_sub = node.subscribe("/MassMatrix", 1000, massCallback);
  ros::Subscriber fristate_sub = node.subscribe("/FriJointState", 1000, friCallback);
  JointEfforts torque_msg;
  FriJointImpedance jointimp_msg;

  for (size_t ii(0); ii<7; ++ii) {
	torque_msg.efforts.push_back(0.0);
	jointimp_msg.stiffness[ii] = 0.0;
	jointimp_msg.damping[ii] = 0.0;
  }
  torque_pub.publish(torque_msg);
  jointimp_pub.publish(jointimp_msg);



  //UDP camera- with option of no camera
  Position3d rawCamData[56];
  jspace::Matrix camData;

  //EDIT ME to actual camera address
  char local_cam_port [10];
  sprintf(local_cam_port, "%d", CLIENT_PORT+2);
  char cam_port [10];
  sprintf(cam_port, "%d", CS8C_PORT+2);
  UdpOSI cameraUDP("50000", "192.168.1.39", cam_port, 0);
  int bytes =0;
  if (cam) {
    bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
    if (bytes<1) {
      fprintf(stderr, "Error receiving data from camera");
	 servo.cleanup();
	 return 0;
    }
    camData = Matrix::Zero(bytes/sizeof(Position3d),3);
    for (size_t ii=0; ii < bytes/sizeof(Position3d); ++ii){
      camData(ii,0) = rawCamData[ii].x*1e-3;
      camData(ii,1) = rawCamData[ii].y*1e-3;
      camData(ii,2) = rawCamData[ii].z*1e-3;
    } 
    Matrix temp(R * camData.transpose() + d*Matrix::Ones(1,bytes/sizeof(Position3d)));
    state.camData_ = temp.transpose();
    cout <<"Recieved initial camera data\n";
    }  

  /*
  //Visualization
  visinfo info;
  UdpOSI visUDP( "57860", const_cast<char*>(CS8C_IPADDR), "50000", 0);*/
  

  int status = servo.init(state);
  if (0!= status) {
    fprintf(stderr, "init callback returned %d\n", status);
    servo.cleanup();
    return 0;
  }

  ros::Time dbg_t0(ros::Time::now());
  ros::Time dump_t0(ros::Time::now());
  ros::Duration dbg_dt(0.1);
  ros::Duration dump_dt(0.05);
 //ros::Rate loop_rate(10);
  ros::Time begin(ros::Time::now());
  while (ros::ok()) {

    //UDP camera in and out
    if (cam) {
      bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
      if (bytes <1) {
	fprintf(stderr,"Error: no camera data recieved");
	servo.cleanup();
	return 0;
      }    
      for (size_t ii=0; ii < bytes/sizeof(Position3d); ++ii){
	camData(ii,0) = rawCamData[ii].x*1e-3;
	camData(ii,1) = rawCamData[ii].y*1e-3;
	camData(ii,2) = rawCamData[ii].z*1e-3;
      }
      Matrix temp(R * camData.transpose() + d*Matrix::Ones(1,bytes/sizeof(Position3d)));
      state.camData_ = temp.transpose();
    }  

    /*
    //Vis out
    for (size_t ii(0); ii<6 ; ++ii) {
      info.q[ii] = state.position_[ii];
    }
    Skill::task_table_t const * tasks(servo.skill->getTaskTable());
    info.R[0] = 0.1;
    info.T[0] = 0.4;//No longer automatic-> should not be manual
    if (cam) {
      for (size_t ii(0); ii < state.camData_.rows(); ++ii) {
	for (size_t jj(0); jj < state.camData_.cols(); ++jj) {
	  info.sp[ii][jj] = state.camData_(ii,jj);
	}
      }
    }

    info.eedes[0] = 0.0;
    info.eedes[1] = 0.0;
    info.eedes[2] = 0.0;
    
    info.cp[0] = 0.0;
    info.cp[1] = 0.0;
    info.cp[2] = 0.0;


    bytes = visUDP.sendPacket((char*)&info, sizeof(info));
     */

    status = servo.update(state, command);
    if (0 != status) {
      fprintf(stderr, "update callback returned %d\n", status);
      servo.cleanup();
      return 0;
    }

    for (size_t ii(0); ii<7; ++ii) {
      if (isnan(command[ii])) {
	torque_msg.efforts[ii] = 0;
      }
      else {
          torque_msg.efforts[ii] = command[ii];
      }
    }

    torque_pub.publish(torque_msg);
    jointimp_pub.publish(jointimp_msg);
    ros::Time t1(ros::Time::now());
    if (verbose) {
      if (t1 - dbg_t0 > dbg_dt) {
	dbg_t0 = t1;
	servo.skill->dbg(cout, "\n\n**************************************************", "");
	controller->dbg(cout, "--------------------------------------------------", "");
	cout << "--------------------------------------------------\n";
	jspace::pretty_print(model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(controller->getCommand(), cout, "gamma", "  ");
	jspace::pretty_print(model->getState().camData_, cout, "cam", "  ");
      }
    }
    if (t1 - dump_t0 > dump_dt) {
      dump_t0 = t1;
      controller->qhlog(*servo.skill, ros::Duration(t1-begin).toSec()*1000);
    }
    ros::spinOnce();
    //loop_rate.sleep();
  }
  
  warnx("shutting down");
  ros::shutdown();
  servo.cleanup();
  //cameraUDP.~UdpOSI();
}
