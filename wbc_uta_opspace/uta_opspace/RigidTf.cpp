
#include "RigidTf.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

#include <Eigen/LU>
#include <Eigen/SVD>

#include <ros/ros.h>

using boost::shared_ptr;

namespace uta_opspace {


  RigidTf::
  RigidTf(std::string const & name)
    : Skill(name),
      ee_task_(0),
      cur_row_(0),
      tfdone_(false)
    {
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("samples", &samples_);
      declareParameter("frequency", &frequency_);
  }

 Status RigidTf::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    

   task_table_.push_back(ee_task_);
   task_table_.push_back(posture_);
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   robotPos = Matrix::Zero(samples_,3);
   camPos = Matrix::Zero(samples_,3);
   last = ros::Time::now().toSec();
   return st;
  }

 Status RigidTf::
  update(Model const & model)
  {
    Status st;    
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
 
    if (ros::Time::now().toSec() - last > 1/frequency_) {
      if (cur_row_ < samples_-1) {
        last = ros::Time::now().toSec();
	++cur_row_;
	//Save data - assumes one point sent
	robotPos(cur_row_-1,0) = ee_task_->getActual()[0];
	robotPos(cur_row_-1,1) = ee_task_->getActual()[1];
	robotPos(cur_row_-1,2) = ee_task_->getActual()[2];

	camPos(cur_row_-1,0) = model.getState().camData_(0,0);
	camPos(cur_row_-1,1) = model.getState().camData_(0,1);
	camPos(cur_row_-1,2) = model.getState().camData_(0,2);

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

	//Find centroids
	Vector robotCent(Vector::Zero(3));
	Vector camCent(Vector::Zero(3));
	for (size_t ii(0); ii < samples_; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    robotCent(jj) = robotCent(jj) + robotPos(ii,jj);
	    camCent(jj) = camCent(jj) + camPos(ii,jj);
	  }
	}
	robotCent = robotCent/(samples_);
	camCent = camCent/(samples_);

	//Subtract centroids from points
	for (size_t ii(0); ii < samples_; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    robotPos(ii,jj) = robotPos(ii,jj) - robotCent(jj);
	    camPos(ii,jj) = camPos(ii,jj) - camCent(jj);
	  }
	}

	//Calculate H
	Matrix H(Matrix::Zero(3,3));
	for (size_t ii(0); ii < 3; ++ii) {
	  for (size_t jj(0); jj < 3; ++jj) {
	    for (size_t kk(0); kk < samples_; ++kk) {
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

	jspace::pretty_print(R*model.getState().camData_.block(0,0,1,3).tranpose()+d, "cam", " ");
	jspace::pretty_print(ee_task_->getActual(), "actual", " ");
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
