
#include "SurfaceMotion.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  SurfaceMotion::
  SurfaceMotion(std::string const & name)
    : Skill(name),
      surface_task_(0),
      ee_task_(0),
      posture_(0),
      ee_pos_(Vector::Zero(3)),
      posture_pos_(Vector::Zero(6)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0),
      forward_(true)
    {
      declareSlot("surface", &surface_task_);
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("eepos", &ee_pos_);
      declareParameter("posture", &posture_pos_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
      declareParameter("ee_num", &ee_num_);

  }

 Status SurfaceMotion::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (0 != ee_pos_.rows()%ee_num_){
      st.ok = false;
      st.errstr = "wrong number of rows in ee_pos vector";
      return st;
    }

    if (0 != posture_pos_.rows()%model.getUnconstrainedNDOF() ){
      st.ok = false;
      st.errstr = "wrong number of rows in ee_pos vector";
      return st;
    }

    if (0 > threshold_) {
      threshold_ = 0.08;
    }
    
    if (0 > vel_threshold_) {
      vel_threshold_ = 0.08;
    }

    ee_goal_ = ee_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! ee_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_task";
      return st;
    }

    posture_goal_ = posture_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! posture_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in posture";
      return st;
    }

    task_table_.push_back(surface_task_);
    task_table_.push_back(ee_task_);
    task_table_.push_back(posture_);
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_eepos(Vector::Zero(ee_num_));
   for (size_t ii(0); ii < ee_num_; ++ii) {
     cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
   }

   st = ee_goal_->set(cur_eepos);
   if (!st) { return st; }

   size_t ndof(model.getUnconstrainedNDOF());

   Vector cur_posture(Vector::Zero(ndof));
   for(size_t jj(0); jj<ndof; jj++) {
     cur_posture[jj] = posture_pos_[ndof*cur_row_+jj];
   }

   st = posture_goal_->set(cur_posture);
   if (!st) { return st; }

   return st;
  }

 Status SurfaceMotion::
  update(Model const & model)
  {
    Status st;    
    Vector cur_eepos(Vector::Zero(ee_num_));
    Vector delta;
    Vector v_delta;

    size_t ndof(model.getUnconstrainedNDOF());

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }
    for(int ii=0; ii<ee_num_; ii++) {
      cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
    }
    
    delta = cur_eepos - ee_task_->getActual();
    jspace::Constraint* constraint = model.getConstraint();
    if (constraint) {
      jspace::State fullState(model.getNDOF(),model.getNDOF(),6);
      constraint->getFullState(model.getState(),fullState);
      v_delta = ee_task_->getJacobian()*fullState.velocity_;
    }
    else {
      v_delta = ee_task_->getJacobian()*model.getState().velocity_;
    }
    
    if (delta.norm() < threshold_ && v_delta.norm() < vel_threshold_) {
      if(forward_) {
	if (cur_row_ < (ee_pos_.rows()/ee_num_)-1) {
	  ++cur_row_;
	  for(size_t jj(0); jj<ee_num_; ++jj) {
	    cur_eepos[jj] = ee_pos_[ee_num_*cur_row_+jj];
	  }
	  st = ee_goal_->set(cur_eepos);
	  if (! st) { return st; } 
	  Vector cur_posture(Vector::Zero(ndof));
	  for(size_t ii(0); ii<ndof; ++ii) {
	    cur_posture[ii] = posture_pos_[ndof*cur_row_+ii];
	  }
	  st = posture_goal_->set(cur_posture);
	  if (! st) { return st; } 
	}
	else { forward_ = false; }
      }
      else {
	if (cur_row_ > 0) {
	  --cur_row_;
	  for(size_t jj(0); jj<ee_num_; ++jj) {
	    cur_eepos[jj] = ee_pos_[ee_num_*cur_row_+jj];
	  }
	  st = ee_goal_->set(cur_eepos);
	  if (! st) { return st; } 
	  Vector cur_posture(Vector::Zero(ndof));
	  for(size_t ii(0); ii<ndof; ++ii) {
	    cur_posture[ii] = posture_pos_[ndof*cur_row_+ii];
	  }
	  st = posture_goal_->set(cur_posture);
	  if (! st) { return st; } 
	}
	else { forward_ = true; }
      }
    }
 
      return st;
    }
    
    Skill::task_table_t const * SurfaceMotion::
      getTaskTable()
    {
      return &task_table_;
    }
    
    
  Status SurfaceMotion::
      checkJStarSV(Task const * task, Vector const & sv)
    {
      /*
	if (sv.rows() != 3) {
	return Status(false, "dimension mismatch");
	}
    if (sv[2] < task->getSigmaThreshold()) {
    return Status(false, "singular");
    }*/
      Status ok;
    return ok;
  }


  void SurfaceMotion::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os,title,prefix);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      task_table_[ii]->dump(os, "", prefix + "  ");
    }
  }
}