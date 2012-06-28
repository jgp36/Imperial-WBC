/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Josh Petersen
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef UTA_RIGID_TF_HPP
#define UTA_RIGID_TF_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class RigidTf
    : public Skill
  {
  public:
    RigidTf(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:
    
    TestPositionOrientationTask * ee_task_;
    task_table_t task_table_;
    
    Parameter * ee_goal_;
    Parameter * ori_goal_x_;
    Parameter * ori_goal_y_;
    Parameter * ori_goal_z_;
    
    Vector ee_pos_;
    Vector ori_x_;
    Vector ori_y_;
    Vector ori_z_;
    double threshold_;
    double vel_threshold_;
    int cur_row_;
    bool tfdone_;

    Matrix robotPos;
    Matrix camPos;

    Matrix R;
    Vector d;
  };
  
}

#endif // UTA_RIGID_TF_HPP
