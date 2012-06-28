/* modified from ControllerVel.cpp*/

#include "ControllerVel.hpp"
#include <jspace/pseudo_inverse.hpp>

// hmm...
#include <Eigen/LU>
#include <Eigen/SVD>
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using jspace::pretty_print;
using boost::shared_ptr;

namespace uta_opspace {
  
  ControllerVel::
  ControllerVel(std::string const & name)
    : Controller(name),
      fallback_(false),
      loglen_(-1),
      logsubsample_(-1),
      logprefix_(""),
      logcount_(-1)
      /*loglen_(1000),
      logsubsample_(1),
      logprefix_("Ramp_Experiment"),
      logcount_(0)*/
  {
    declareParameter("loglen", &loglen_, PARAMETER_FLAG_NOLOG);
    declareParameter("logsubsample", &logsubsample_, PARAMETER_FLAG_NOLOG);
    declareParameter("logprefix", &logprefix_, PARAMETER_FLAG_NOLOG);
    declareParameter("jpos", &jpos_);
    declareParameter("jvel", &jvel_);
    declareParameter("gamma", &gamma_);
    declareParameter("fullJpos", &fullJpos_);
    declareParameter("fullJvel", &fullJvel_);
  }
  
  
  Status ControllerVel::
  check(std::string const * param, std::string const & value) const
  {
    if ((param == &logprefix_) && (loglen_ > 0)) {
      // could be smart and check if another log is oVeloiVel, but let's
      // just abort that
      logcount_ = 0;
    }
    Status ok;
    return ok;
  }
  
  
  void ControllerVel::
  setFallbackTask(shared_ptr<Task> task)
  {
    fallback_task_ = task;
  }
  
  
  Status ControllerVel::
  init(Model const & model)
  {
    if ( ! fallback_task_) {
      JPosTrjTask * pt(new JPosTrjTask("ControllerVel_fallback_posture"));
      pt->quickSetup(0.01, 50, 5, 1, 2);
      fallback_task_.reset(pt);
    }
    if ( ! dynamic_cast<JPosTrjTask*>(fallback_task_.get())) {
      return Status(false, "fallback task has to be a posture (for now)");
    }
    return Status();
  }
  
  
  Status ControllerVel::
  computeFallback(Model const & model,
		  bool init_required,
		  Vector & gamma)
  {
    Status st;
    
    if (init_required) {
      st = fallback_task_->init(model);
      if ( ! st) {
	return Status(false, "fallback task failed to initialize: " + st.errstr);
      }
    }
    
    st = fallback_task_->update(model);
    if ( ! st) {
      return Status(false, "fallback task failed to update: " + st.errstr);
    }
    
    gamma = fallback_task_->getCommand();
    
    // logging and debug
    jpos_ = model.getState().position_;
    jvel_ = model.getState().velocity_;
    gamma_ = gamma;
    
    return st;
  }
  
  
  Status ControllerVel::
  computeCommand(Model const & model,
		 Skill & skill,
		 Vector & gamma)
  {
    //////////////////////////////////////////////////
    // Shortcut if we are already in fallback mode. Later, we'll have
    // to code a way to get out of fallback, but for now it's a
    // one-way ticket.
    if (fallback_) {
      return computeFallback(model, false, gamma);
    }
    //////////////////////////////////////////////////
    
    Status st(skill.update(model));
    if ( ! st) {
      fallback_ = true;
      fallback_reason_ = "skill update failed: " + st.errstr;
      return computeFallback(model, true, gamma);
    }
    
    Skill::task_table_t const * tasks(skill.getTaskTable());
    if ( ! tasks) {
      fallback_ = true;
      fallback_reason_ = "null task table";
      return computeFallback(model, true, gamma);
    }
    if (tasks->empty()) {
      fallback_ = true;
      fallback_reason_ = "empty task table";
      return computeFallback(model, true, gamma);
    }
    
    //////////////////////////////////////////////////
    // the magic nullspace sauce...


    fullJpos_ = model.getFullState().position_;
    fullJvel_ = model.getFullState().velocity_;
    

    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      return Status(false, "failed to retrieve inverse mass inertia");
    }

    jspace::Constraint * constraint = model.getConstraint();

    if (constraint) {
      if(!constraint->updateJc(model)) {
	return Status(false, "failed to update Jc");
      }
    }

    Matrix Nc;
    if (constraint) {
      if (!constraint->getNc(ainv,Nc)) {
	return Status(false, "failed to get Nc");
      }
    }
    else {
      Nc = Matrix::Identity(model.getNDOF(),model.getNDOF());
    }

    Matrix UNc;
    if (constraint) {
      Matrix U;
      if (!constraint->getU(U)) {
	return Status(false, "failed to get U");
      }
      UNc = U*Nc;
    }
    else {
      UNc = Matrix::Identity(model.getNDOF(),model.getNDOF());
    }  

    Matrix phi(UNc * ainv * UNc.transpose());
    

    Matrix UNcBar;
    if (constraint) {
      Matrix phiinv;
      //XXXX hardcoded sigma threshold
      jspace::pseudoInverse(phi,
		    0.0001,
		    phiinv, 0);
      UNcBar = ainv * UNc.transpose() * phiinv;
    }
    else {
      UNcBar = Matrix::Identity(model.getNDOF(),model.getNDOF());
    }

    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(tasks->size() - 1);
    Matrix nstar(Matrix::Identity(model.getUnconstrainedNDOF(), model.getUnconstrainedNDOF()));
    int first_active_task_index(0); // because tasks can have empty Jacobian
    
    for (size_t ii(0); ii < tasks->size(); ++ii) {
      
      Task const * task((*tasks)[ii]);
      Matrix const & jac(task->getJacobian());
      
      // skip inactive tasks at beginning of table
      if ((0 == jac.rows()) || (0 == jac.cols())) {
	++first_active_task_index;
	if (first_active_task_index >= tasks->size()) {
	  return Status(false, "no active tasks (all Jacobians are empty)");
	}
	continue;
      }

      Matrix jstar;
      if (ii == first_active_task_index) {
	jstar = jac * UNcBar;
      }
      else {
	jstar = jac * UNcBar * nstar;
      }

      //Least norm inverse
      Matrix jjt(jstar * jstar.transpose());
      Matrix jstarpinv(jstar.transpose()*jjt.inverse());
      jinv_ = jstarpinv;	
      
      Vector sv_jstar;
      
      if (1 == jjt.rows()) {	// work around limitations of Eigen2 SVD.
	sv_jstar = Vector::Ones(1, 1) * jjt.coeff(0, 0);
      }
      else {
	//////////////////////////////////////////////////
	// If Eigen ends up freeing a non-allocated pointer on the line
	// below: that appears to happen on zero-rank matrices when you
	// do an SVD (at least that's my best guess while writiVel this
	// comment). Maybe you have a task hierarchy with entries after
	// all degrees of freedom have been eaten up, or maybe you have
	// a task more than one in the hierarchy. This should not
	// trigger a segfault, and/or it should be detected earlier, but
	// this effect is a bit obscure for now.
	sv_jstar = Eigen::SVD<Matrix>(jjt).singularValues();
      }
      
      st = skill.checkJStarSV(task, sv_jstar);
      if ( ! st) {
	fallback_ = true;
	fallback_reason_ = "checkJStarSV failed: " + st.errstr;
	return computeFallback(model, true, gamma);
      }

      Vector force(task->getForce());
      if (force.rows() == 0) {
	force = Vector::Zero(jstar.rows());
      }
      
      // could add coriolis-centrifugal just like pstar...
      if (ii == first_active_task_index) {
	// first time around: initialize gamma
	gamma = jstarpinv * task->getCommand();
	actual_ = task->getActual();
      }
      else {
	Vector fcomp;
	// here, gamma is still at the previous iteration's value
	fcomp = jstar * gamma;
	gamma += jstarpinv * (task->getCommand()  - fcomp);
      }
      
      if (ii != n_minus_1) {
	// not sure whether Eigen2 correctly handles the case where a
	// matrix gets updated by left-multiplication...
	Matrix const
	  nnext((Matrix::Identity(model.getUnconstrainedNDOF(), model.getUnconstrainedNDOF()) - jstarpinv * jstar) * nstar);
	nstar = nnext;
      }
    }
    
    if (tasks->size() <= first_active_task_index) {
      fallback_ = true;
      fallback_reason_ = "no active tasks";
      return computeFallback(model, true, gamma);
    }
    
    // loggiVel and debug
    jpos_ = model.getState().position_;
    jvel_ = model.getState().velocity_;
    gamma_ = gamma;
    
    return st;
  }
  
  
  void ControllerVel::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "log count: " << logcount_ << "\n"
       << prefix << "parameters\n";
    dump(os, "", prefix + "  ");
    if (fallback_) {
      os << prefix << "# FALLBACK MODE ENABLED ##########################\n"
	 << prefix << "# reason: " << fallback_reason_ << "\n";
    }
  }
  
  
  void ControllerVel::
  qhlog(Skill & skill, long long timestamp)
  {
    if (0 == logcount_) {
      // initialize loggiVel
      log_.clear();
      log_.push_back(shared_ptr<ParameterLog>(new ParameterLog("ctrl_" + instance_name_,
							       getParameterTable())));
      log_.push_back(shared_ptr<ParameterLog>(new ParameterLog("skill_" + skill.getName(),
							       skill.getParameterTable())));
      Skill::task_table_t const * tasks(skill.getTaskTable());
      if (tasks) {
	for (size_t ii(0); ii < tasks->size(); ++ii) {
	  std::ostringstream nm;
	  nm << "task_" << ii << "_" << (*tasks)[ii]->getName();
	  log_.push_back(shared_ptr<ParameterLog>(new ParameterLog(nm.str(),
								   (*tasks)[ii]->getParameterTable())));
	}
      }
    }
    else if ((0 < loglen_) && (loglen_ == logcount_)) {
      logcount_ = -2;
    }
    if (0 <= logcount_) {
      ++logcount_;
    }
    
    if (0 < logcount_) {
      if ((logsubsample_ <= 0)
	  || (0 == (logcount_ % logsubsample_))) {
	for (size_t ii(0); ii < log_.size(); ++ii) {
	  log_[ii]->update(timestamp);
	}
      }
    }
    
    if (-2 == logcount_) {
      for (size_t ii(0); ii < log_.size(); ++ii) {
	log_[ii]->writeFiles(logprefix_, &std::cerr);
      }
      logcount_ = -1;
    }
  }
  
}
