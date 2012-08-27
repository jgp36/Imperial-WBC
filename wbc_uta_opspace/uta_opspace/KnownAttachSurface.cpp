
#include "KnownAttachSurface.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  KnownAttachSurface::
  KnownAttachSurface(std::string const & name)
    : Skill(name),
      surface_task_(0),
      ee_task_(0),
      zero_(0),
      one_(1)
    {
      declareSlot("surface", &surface_task_);
      declareSlot("eesurf", &ee_surf_task_);
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("surflimitlow", &surflimitlow_);
      declareParameter("surflimithigh", &surflimithigh_);
      declareParameter("camData", &camData_);
  }

 Status KnownAttachSurface::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }

    ffvel_ = surface_task_->lookupParameter("ffvel", PARAMETER_TYPE_REAL);
    ffacc_ = surface_task_->lookupParameter("ffacc", PARAMETER_TYPE_REAL);
    camParam_ = surface_task_->lookupParameter("camData", PARAMETER_TYPE_MATRIX);

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
   ffvel_->set(zero_);
   ffacc_->set(zero_);
   camData_ = *camParam_->getMatrix();
   state_ = STATE_NORMAL;

   return st;
  }

 Status KnownAttachSurface::
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
		ffvel_->set(one_);
		ffacc_->set(one_);
      		state_ = STATE_ATTACHED;
   	 }
	 break;
    case STATE_ATTACHED:
         if (surface_task_->evalPos(model,ee_task_->getActual()) >= surflimithigh_) {
		ffvel_->set(zero_);
		ffacc_->set(zero_);
      		state_ = STATE_NORMAL;
   	 }
	 break;
    }
    
    return st;
  }
  
  Skill::task_table_t const * KnownAttachSurface::
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
  
  
  Status KnownAttachSurface::
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


  void KnownAttachSurface::
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
