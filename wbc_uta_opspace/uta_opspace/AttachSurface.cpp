
#include "AttachSurface.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  AttachSurface::
  AttachSurface(std::string const & name)
    : Skill(name),
      surface_task_(0),
      ori_task_(0),
      ee_task_(0),
      ee_pos_(Vector::Zero(3)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0),
      forward_(true)
    {
      declareSlot("surface", &surface_task_);
      declareSlot("ori", &ori_task_);
      declareSlot("eesurf", &ee_surf_task_);
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("eepos", &ee_pos_);
      declareParameter("defposture", &defposture_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
      declareParameter("ee_num", &ee_num_);
      declareParameter("zoffset", &zoffset_);
      declareParameter("zamp", &zamp_);
      declareParameter("surflimit", &surflimit_);

  }

 Status AttachSurface::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (0 != ee_pos_.rows()%ee_num_){
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

    ee_surf_goal_ = ee_surf_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! ee_surf_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_surf_task";
      return st;
    }

    ee_goal_ = ee_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! ee_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_task";
      return st;
    }

    pos_goal_ = posture_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! pos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in posture";
      return st;
    }

    attached_task_table_.push_back(surface_task_);
    attached_task_table_.push_back(ori_task_);
    attached_task_table_.push_back(ee_surf_task_);

    normal_task_table_.push_back(ee_task_);
    normal_task_table_.push_back(posture_);
     
   for (size_t ii(0); ii < attached_task_table_.size(); ++ii) {
      st = attached_task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   for (size_t ii(0); ii < normal_task_table_.size(); ++ii) {
      st = normal_task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_eepos(Vector::Zero(ee_num_));
   for (size_t ii(0); ii < ee_num_; ++ii) {
     cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
   }

   st = ee_surf_goal_->set(cur_eepos);
   if (!st) { return st; }

   cur_eepos = Vector::Zero(3);
   for (size_t ii(0); ii < ee_num_; ++ii) {
     cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
   }
   cur_eepos[2] = zoffset_ + zamp_;

   st = ee_goal_->set(cur_eepos);
   if (!st) { return st; }

   st = pos_goal_->set(defposture_);
   if (!st) { return st; }


   state_ = STATE_NORMAL;

   return st;
  }

 Status AttachSurface::
  update(Model const & model)
  {
    Status st;    
    Vector cur_eepos(Vector::Zero(3));
    Vector delta;
    Vector v_delta;
    jspace::Constraint* constraint;

    size_t ndof(model.getUnconstrainedNDOF());

    for (size_t ii(0); ii < attached_task_table_.size(); ++ii) {
      st = attached_task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }

    for (size_t ii(0); ii < normal_task_table_.size(); ++ii) {
      st = normal_task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }

    //Update surface calc
    for(int ii=0; ii<ee_num_; ii++) {
      cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
    }
    cur_eepos[2] = zoffset_ + zamp_* cos(M_PI * cur_row_ * ee_num_/ee_pos_.rows());  

    if (surface_task_->evalPos(model,cur_eepos) <= surflimit_) state_ = STATE_ATTACHED;
    else state_ = STATE_NORMAL;


    switch(state_) {
    case STATE_NORMAL:    
      delta = cur_eepos - ee_task_->getActual();
      constraint = model.getConstraint();
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
	    cur_eepos[2] = zoffset_ + zamp_* cos(M_PI * cur_row_ * ee_num_/ee_pos_.rows());
	    st = ee_goal_->set(cur_eepos);
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
	    cur_eepos[2] = zoffset_ + zamp_* cos(M_PI * cur_row_ * ee_num_/ee_pos_.rows());
	    st = ee_goal_->set(cur_eepos);
	    if (! st) { return st; } 
	  }
	  else { forward_ = true; }
	}
      }
      break;
      
    case STATE_ATTACHED:
      cur_eepos = Vector::Zero(ee_num_);
      for(int ii=0; ii<ee_num_; ii++) {
	cur_eepos[ii] = ee_pos_[ee_num_*cur_row_+ii];
      }
      
      delta = cur_eepos - ee_surf_task_->getActual();
      constraint = model.getConstraint();
      if (constraint) {
	jspace::State fullState(model.getNDOF(),model.getNDOF(),6);
	constraint->getFullState(model.getState(),fullState);
	v_delta = ee_surf_task_->getJacobian()*fullState.velocity_;
      }
      else {
	v_delta = ee_surf_task_->getJacobian()*model.getState().velocity_;
      }
      
      if (delta.norm() < threshold_ && v_delta.norm() < vel_threshold_) {
	if(forward_) {
	  if (cur_row_ < (ee_pos_.rows()/ee_num_)-1) {
	    ++cur_row_;
	    for(size_t jj(0); jj<ee_num_; ++jj) {
	      cur_eepos[jj] = ee_pos_[ee_num_*cur_row_+jj];
	    }
	    st = ee_surf_goal_->set(cur_eepos);
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
	    st = ee_surf_goal_->set(cur_eepos);
	    if (! st) { return st; } 
	  }
	  else { forward_ = true; }
	}
      }
      break;
      
    }
 
      return st;
    }
    
    Skill::task_table_t const * AttachSurface::
      getTaskTable()
    {
      switch(state_) {
      case STATE_NORMAL:
	return &normal_task_table_;
      case STATE_ATTACHED:
	return &normal_task_table_;
      }
      return 0;
    }
    
    
  Status AttachSurface::
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


  void AttachSurface::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os,title,prefix);
    switch (state_) {
    case STATE_ATTACHED:
      for (size_t ii(0); ii < attached_task_table_.size(); ++ii) {
	attached_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      break;
    case STATE_NORMAL:
      for (size_t ii(0); ii < normal_task_table_.size(); ++ii) {
	normal_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      break;
    }

  }
}
