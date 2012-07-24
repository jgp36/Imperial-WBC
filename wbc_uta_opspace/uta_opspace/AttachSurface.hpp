

#ifndef UTA_OPSPACE_ATTACH_SURFACE_HPP
#define UTA_OPSPACE_ATTACH_SURFACE_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class AttachSurface
    : public Skill
  {
  public:
    AttachSurface(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:

    enum {
	STATE_ATTACHED,
	STATE_NORMAL
    } state_;
    
    TestImplicitSurfaceTask * surface_task_;
    TestOriSurfaceTask * ori_task_;
    TestPureSelectCartPosTask * ee_surf_task_;

    PureCartPosTask * ee_task_;
    PureJPosTask * posture_;

    task_table_t normal_task_table_;
    task_table_t attached_task_table_;
    
    Parameter * ee_surf_goal_;
    Parameter * ee_goal_;
    Parameter * pos_goal_;
    
    Vector ee_pos_;
    Vector defposture_;
    int ee_num_;
    double threshold_;
    double vel_threshold_;
    int cur_row_;
    bool forward_;

    double zoffset_;
    double zamp_;

    double surflimit_;
    
  };
  
}

#endif // UTA_OPSPACE_ATTACH_SURFACE_HPP
