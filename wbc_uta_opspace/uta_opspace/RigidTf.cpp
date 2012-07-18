
#include "RigidTf.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

#include <Eigen/LU>
#include <Eigen/SVD>

using boost::shared_ptr;

namespace uta_opspace {


  RigidTf::
  RigidTf(std::string const & name)
    : Skill(name),
      ee_task_(0),
      ee_pos_(Vector::Zero(3)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0),
      tfdone_(false)
    {
      declareSlot("eepos", &ee_task_);
      declareParameter("eepos", &ee_pos_);
      declareParameter("ori_x", &ori_x_);
      declareParameter("ori_y", &ori_y_);
      declareParameter("ori_z", &ori_z_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
  }

 Status RigidTf::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (0 != ee_pos_.rows()%3){
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
    ori_goal_x_ = ee_task_->lookupParameter("goal_x", PARAMETER_TYPE_VECTOR);
    ori_goal_y_ = ee_task_->lookupParameter("goal_y", PARAMETER_TYPE_VECTOR);
    ori_goal_z_ = ee_task_->lookupParameter("goal_z", PARAMETER_TYPE_VECTOR);

    if ( ! ee_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_task";
      return st;
    }

   if ( ! ori_goal_x_) {
      st.ok = false;
      st.errstr = "no appropriate ori x goal parameter in ee_task";
      return st;
    }
   if ( ! ori_goal_y_) {
      st.ok = false;
      st.errstr = "no appropriate ori y goal parameter in ee_task";
      return st;
    }
   if ( ! ori_goal_z_) {
      st.ok = false;
      st.errstr = "no appropriate ori z goal parameter in ee_task";
      return st;
   }

   task_table_.push_back(ee_task_);
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_eepos(Vector::Zero(3));
   for (size_t ii(0); ii < 3; ++ii) {
     cur_eepos[ii] = ee_pos_[3*cur_row_+ii];
   }

   st = ee_goal_->set(cur_eepos);
   if (!st) { return st; }

   st = ori_goal_x_->set(ori_x_);
   if (!st) { return st; }
   st = ori_goal_y_->set(ori_y_);
   if (!st) { return st; }
   st = ori_goal_z_->set(ori_z_);
   if (!st) { return st; }

   robotPos = Matrix::Zero(ee_pos_.rows()/3,3);
   camPos = Matrix::Zero(ee_pos_.rows()/3,3);
   return st;
  }

 Status RigidTf::
  update(Model const & model)
  {
    Status st;    
    Vector cur_eepos(Vector::Zero(3));
    Vector delta;
    Vector v_delta;

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }
    for(int ii=0; ii<3; ii++) {
      cur_eepos[ii] = ee_pos_[3*cur_row_+ii];
    }
    
    delta = cur_eepos - ee_task_->getActual().block(0, 0, 3, 1);
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
      if (cur_row_ < (ee_pos_.rows()/3)-1) {
	++cur_row_;
	for(size_t jj(0); jj<3; ++jj) {
	  cur_eepos[jj] = ee_pos_[3*cur_row_+jj];
	}
	st = ee_goal_->set(cur_eepos);
	if (! st) { return st; } 
	//Save data - assumes one point sent
	robotPos(cur_row_-1,0) = ee_task_->getActual()[0];
	robotPos(cur_row_-1,1) = ee_task_->getActual()[1];
	robotPos(cur_row_-1,2) = ee_task_->getActual()[2];

	camPos(cur_row_-1,0) = model.getState().camData_(0,0);
	camPos(cur_row_-1,1) = model.getState().camData_(0,1);
	camPos(cur_row_-1,2) = model.getState().camData_(0,2);

	if (camPos(cur_row_-1,0) < -100) {
	   st.ok = false;
      	   st.errstr = "Marker Missing";
      	   return st;
	}
      }
      //Calculate Transform
      else if (tfdone_ == false) { 

	//Save last data
	robotPos(cur_row_,0) = ee_task_->getActual()[0];
	robotPos(cur_row_,1) = ee_task_->getActual()[1];
	robotPos(cur_row_,2) = ee_task_->getActual()[2];

	camPos(cur_row_,0) = model.getState().camData_(0,0);
	camPos(cur_row_,1) = model.getState().camData_(0,1);
	camPos(cur_row_,2) = model.getState().camData_(0,2);

	if (camPos(cur_row_,0) < -100) {
	   st.ok = false;
      	   st.errstr = "Marker Missing";
      	   return st;
	}

	//Find centroids
	Vector robotCent(Vector::Zero(3));
	Vector camCent(Vector::Zero(3));
	for (size_t ii(0); ii < ee_pos_.rows()/3; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    robotCent(jj) = robotCent(jj) + robotPos(ii,jj);
	    camCent(jj) = camCent(jj) + camPos(ii,jj);
	  }
	}
	robotCent = robotCent/(ee_pos_.rows()/3);
	camCent = camCent/(ee_pos_.rows()/3);

	//Subtract centroids from points
	for (size_t ii(0); ii < ee_pos_.rows()/3; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    robotPos(ii,jj) = robotPos(ii,jj) - robotCent(jj);
	    camPos(ii,jj) = camPos(ii,jj) - camCent(jj);
	  }
	}

	//Calculate H
	Matrix H(Matrix::Zero(3,3));
	for (size_t ii(0); ii < 3; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    for (size_t kk(0); kk < ee_pos_.rows()/3; ++kk) {
	      H(ii,jj) = H(ii,jj) + camPos(kk,ii)*robotPos(kk,jj);
	    }
	  }
	}

	//Calculate R from SVD of H
	R = Eigen::SVD<Matrix>(H).matrixV()*Eigen::SVD<Matrix>(H).matrixU().transpose();

	//Calculate d
	d = robotCent - R*camCent;

	jspace::pretty_print(R,std::cout, "R", "  ");
	jspace::pretty_print(d,std::cout, "d", "  ");
	
	tfdone_ = true;
      }
     else {
	jspace::pretty_print(R,std::cout, "R", "  ");
	jspace::pretty_print(d,std::cout, "d", "  ");
     }
    }
 
      return st;
    }
    
    Skill::task_table_t const * RigidTf::
      getTaskTable()
    {
      return &task_table_;
    }
    
    
  Status RigidTf::
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


  void RigidTf::
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
