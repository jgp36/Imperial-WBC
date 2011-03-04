/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen and Luis Sentis
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opspace/behavior_library.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  TPBehavior::
  TPBehavior(std::string const & name)
    : Behavior(name)
  {
    declareSlot("default", "eepos", &eepos_);
    declareSlot("default", "posture", &posture_);
  }
  
  
  Status TPBehavior::
  init(Model const & model)
  {
    Status st(Behavior::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TPBehavior::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Status const st(task_table_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
  
  Behavior::task_table_t const * TPBehavior::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TPBehavior::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (task == eepos_) {
      if (sv.rows() != 3) {
	return Status(false, "eepos dimension mismatch");
      }
      if (sv[2] < eepos_->getSigmaThreshold()) {
	return Status(false, "singular eepos");
      }
    }
    Status ok;
    return ok;
  }
  
}
