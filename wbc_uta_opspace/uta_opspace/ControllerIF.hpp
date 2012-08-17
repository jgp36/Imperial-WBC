/*Special controller for Implicit Surface Control
Operates normally unless Orientation Control is detected for the secondary task
Then assumes a hierachy of the form
Task 1: Implicit Surface
Task 2: Orientation
Task 3: Position
Task 4: Posture(Optional)
*/

#ifndef UTA_OPSPACE_CONTROLLER_IF_HPP
#define UTA_OPSPACE_CONTROLLER_IF_HPP

#include <opspace/Controller.hpp>
#include <boost/shared_ptr.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class ControllerIF
    : public Controller
  {
  public:
    explicit ControllerIF(std::string const & name);
    
    void setFallbackTask(boost::shared_ptr<Task> task);
    
    virtual Status init(Model const & model);

    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma);
    
    virtual Status check(std::string const * param, std::string const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
    Status computeFallback(Model const & model,
			   bool init_required,
			   Vector & gamma);
    
    inline Vector const & getCommand() const { return gamma_; }

    inline Vector const & getActual() const { return actual_; }
    
    void qhlog(Skill & skill, long long timestamp);

    void setAmatrix(Matrix A);

    void setgrav(Vector g);
    
    
  protected:
    boost::shared_ptr<Task> fallback_task_;
    ////    std::vector<Vector> sv_lstar_; // stored only for dbg()
    bool fallback_;
    std::string fallback_reason_;
    
    std::vector<boost::shared_ptr<ParameterLog> > log_;
    int loglen_;		// <= 0 means disabled
    int logsubsample_;
    std::string logprefix_;
    
    // -1 means off, 0 means init, -2 means maybeWriteLogFiles() will
    // actually write them (this gets set when ==loglen_)
    mutable int logcount_;
    
    // for logging and debugging via Parameter tools, don't bother to
    // implement check() methods because one day real soon now we'll
    // be able to flag parameters as read-only and then the superclass
    // can take care of signaling errors when someone writes to
    // them...
    Vector jpos_;
    Vector jvel_;
    Vector gamma_;

    Vector actual_;

    Vector fullJpos_;
    Vector fullJvel_;

    Matrix A_;
    bool a_set;
    Vector g_;
    bool g_set;
  };

}

#endif // UTA_OPSPACE_CONTROLLER_IF_HPP
