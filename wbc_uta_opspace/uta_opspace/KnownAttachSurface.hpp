

#ifndef UTA_OPSPACE_KNOWN_ATTACH_SURFACE_HPP
#define UTA_OPSPACE_KNOWN_ATTACH_SURFACE_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class KnownAttachSurface
    : public Skill
  {
  public:
    KnownAttachSurface(std::string const & name);
    
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
    
    TestKnownImplicitSurfaceTask * surface_task_;
    TestPureSelectCartPosTask * ee_surf_task_;
    PureCartPosTask * ee_task_;
    PureJPosTask * posture_;

    Parameter * ffacc_;
    Parameter * ffvel_;
    Parameter * camParam_;

    jspace::Matrix camData_;

    task_table_t normal_task_table_;
    task_table_t attached_task_table_;
    
    double surflimitlow_;
    double surflimithigh_;

    double one_;
    double zero_;
    
  };
  
}

#endif // UTA_OPSPACE_ATTACH_SURFACE_HPP
