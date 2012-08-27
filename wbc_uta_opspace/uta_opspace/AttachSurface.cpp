
#include "AttachSurface.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  AttachSurface::
  AttachSurface(std::string const & name)
    : Skill(name),
      surface_task_(0),
      ee_task_(0)
    {
      declareSlot("surface", &surface_task_);
      declareSlot("eesurf", &ee_surf_task_);
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("surflimitlow", &surflimitlow_);
      declareParameter("surflimithigh", &surflimithigh_);
  }

 Status AttachSurface::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }


    attached_task_table_.push_back(surface_task_);
    attached_task_table_.push_back(ee_surf_task_);
    attached_task_table_.push_back(posture_);

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

   state_ = STATE_NORMAL;

   return st;
  }

 Status AttachSurface::
  update(Model const & model)
  {
    Status st;    
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


   switch(state_)  {
    case STATE_NORMAL:
         if (surface_task_->evalPos(model,ee_task_->getActual()) <= surflimitlow_) {
      		state_ = STATE_ATTACHED;
   	 }
	 break;
    case STATE_ATTACHED:
         if (surface_task_->evalPos(model,ee_task_->getActual()) >= surflimithigh_) {
      		state_ = STATE_NORMAL;
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
      return &attached_task_table_;
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
    //Skill::dbg(os,title,prefix);
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
