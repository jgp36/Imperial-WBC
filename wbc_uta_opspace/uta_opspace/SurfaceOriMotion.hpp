

#ifndef UTA_OPSPACE_SURFACE_ORI_MOTION_POS_HPP
#define UTA_OPSPACE_SURFACE_ORI_MOTION_POS_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class SurfaceOriMotion
    : public Skill
  {
  public:
    SurfaceOriMotion(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:
    
    TestImplicitSurfaceTask * surface_task_;
    TestVelOriSurfaceTask * ori_task_;
    TestPureSelectCartPosTask * ee_task_;
    task_table_t task_table_;
    
    Parameter * ee_goal_;
    
    Vector ee_pos_;
    int ee_num_;
    double threshold_;
    double vel_threshold_;
    int cur_row_;
    bool forward_;
    
  };
  
}

#endif // UTA_OPSPACE_CART_MULTI_POS_HPP
