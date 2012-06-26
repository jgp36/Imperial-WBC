/* based off servo.cpp in wbc_m3_ctrl
*/

#include <wbc_stb/rt_util.h>

#include <ros/ros.h>
#include <jspace/test/sai_util.hpp>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <uta_opspace/ControllerVel.hpp>
#include <wbc_core/opspace_param_callbacks.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>
#include <sstream>
#include <string>

#include <wbc_stb/udp_osi.h>
#include <wbc_stb/cs8c_interface.h>
#include <wbc_stb/ndtypes.h>

using namespace wbc_stb;
using namespace opspace;
using namespace wbc_core_opspace;
using namespace uta_opspace;
using namespace boost;
using namespace std;


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
static shared_ptr<ControllerVel> controller;


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
  
  
  ros::init(argc, argv, "wbc_stb_servo", ros::init_options::NoSigintHandler);
  parse_options(argc, argv);
  ros::NodeHandle node("~");

  jspace::State state(6, 6, 6);
  jspace::Vector command(6);
  
  controller.reset(new ControllerVel("wbc_stb::servo"));
  param_cbs.reset(new ParamCallbacks());
  Servo servo;
  if (verbose) {
    warnx("initializing param callbacks");
  }
  registry.reset(factory->createRegistry());
  registry->add(controller);
  param_cbs->init(node, registry, 1, 100);

  //UDP robot
  char local_port [5];
  sprintf(local_port, "%d", CLIENT_PORT);
  char robot_port [5];
  sprintf(robot_port, "%d", CS8C_PORT);
  UdpOSI robotUDP( local_port, const_cast<char*>(CS8C_IPADDR), robot_port, 0);


  //Initial message
  CS8CJntVelCmd robotCmd;
  for (size_t ii(0); ii < JNT_VEL_LEN; ++ii) {
    robotCmd.jntVel[ii] = 0;
  }
  robotCmd.pktID = JNT_VEL_CMD_ID;
  int bytes = robotUDP.sendPacket((char*)&robotCmd, sizeof(robotCmd));
  if (bytes != 0) {
    fprintf(stderr, "Failed to send robot UDP");
    servo.cleanup();
    return 0;
  }
  cout <<"Sent initial command packet\n";

  //Wait for a response
  CS8CJntFbk robotFb;
  bytes = 0;
  while (bytes <1){
    bytes = robotUDP.recvPacket((char*)&robotFb, sizeof(robotFb));
  }
  if (robotFb.cs8cErrno != 0) {
    fprintf(stderr, "Error with feedback packet: %d",robotFb.cs8cErrno);
    servo.cleanup();
    return 0;
  }
  if (robotFb.pktID != JNT_FBK_ID) {
    fprintf(stderr, "Error: wrong feedback packet. Switch to joint feedback");
    servo.cleanup();
    return 0;
  }
  for (size_t ii(0); ii < JNT_POS_LEN; ++ii) {
    state.position_[ii] = robotFb.jntPos[ii];
    state.velocity_[ii] = robotFb.jntVel[ii];
  }
  cout <<"Recieved initial robot states\n";

  //UDP camera- with option of no camera
  Position3d rawCamData[10];
  jspace::Matrix camData;
  //EDIT ME to actual camera address
  UdpOSI cameraUDP(local_port, const_cast<char*>(CS8C_IPADDR), robot_port, 0);
  if (cam) {
    bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
    camData = Matrix::Zero(bytes/sizeof(rawCamData),3);
    for (size_t ii=0; ii < bytes/sizeof(rawCamData); ++ii){
      camData(ii,1) = rawCamData[ii].x;
      camData(ii,2) = rawCamData[ii].y;
      camData(ii,3) = rawCamData[ii].z;
    }
    state.camData_ = camData;
    cout <<"Recieved initial camera data\n";
  }  

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
  while (ros::ok()) {

    //UDP robot in
    bytes = robotUDP.recvPacket((char*)&robotFb, sizeof(robotFb));
    if (bytes <1) {
      fprintf(stderr,"Error: no robot states recieved");
      servo.cleanup();
      return 0;
    }
    if (robotFb.cs8cErrno != 0) {
      fprintf(stderr, "Error with feedback packet: %d",robotFb.cs8cErrno);
      servo.cleanup();
      return 0;
    }
    if (robotFb.pktID != JNT_FBK_ID) {
      fprintf(stderr, "Error: wrong feedback packet. Switch to joint feedback");
      servo.cleanup();
      return 0;
    }
    for (size_t ii(0); ii < JNT_POS_LEN; ++ii) {
      state.position_[ii] = robotFb.jntPos[ii];
      state.velocity_[ii] = robotFb.jntVel[ii];
    }

    //UDP camera in
    if (cam) {
      bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
      if (bytes <1) {
	fprintf(stderr,"Error: no camera data recieved");
	servo.cleanup();
	return 0;
      }    
      for (size_t ii=0; ii < bytes/sizeof(rawCamData); ++ii){
	camData(ii,1) = rawCamData[ii].x;
	camData(ii,2) = rawCamData[ii].y;
	camData(ii,3) = rawCamData[ii].z;
      }
      state.camData_ = camData;
    }  

    status = servo.update(state, command);
    if (0 != status) {
      fprintf(stderr, "update callback returned %d\n", status);
      servo.cleanup();
      return 0;
    }

    //UDP robot out
    for (size_t ii(0); ii < JNT_VEL_LEN; ++ii) {
      robotCmd.jntVel[ii] = command[ii];
    }
    bytes = robotUDP.sendPacket((char*)&robotCmd, sizeof(robotCmd));
    if (bytes != 0) {
      fprintf(stderr, "Failed to send robot UDP");
      servo.cleanup();
      return 0;
    }

    ros::Time t1(ros::Time::now());
    if (verbose) {
      if (t1 - dbg_t0 > dbg_dt) {
	dbg_t0 = t1;
	servo.skill->dbg(cout, "\n\n**************************************************", "");
	controller->dbg(cout, "--------------------------------------------------", "");
	cout << "--------------------------------------------------\n";
	jspace::pretty_print(model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(model->getState().velocity_, cout, "jvel", "  ");
	jspace::pretty_print(model->getState().force_, cout, "jforce", "  ");
	jspace::pretty_print(controller->getCommand(), cout, "gamma", "  ");
      }
    }
    if (t1 - dump_t0 > dump_dt) {
      dump_t0 = t1;
      //controller->qhlog(*servo.skill, rt_get_cpu_time_ns() / 1000);
    }
    ros::spinOnce();
  }
  
  warnx("shutting down");
  ros::shutdown();
  servo.cleanup();
  robotUDP.~UdpOSI();
  cameraUDP.~UdpOSI();
}
