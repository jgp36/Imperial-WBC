#include <jspace/State.hpp>

#ifndef WBC_FRI_VEL_EST_H
#define WBC_FRI_VEL_EST_H

namespace wbc_fri {
  class vel_est {
  public:
    vel_est() {};
    void init(double R, double rj, int m, double pos, double t);
    double update(double pos, double t);

  private:
    double R_;
    double rj_;
    int m_;
    double sj_;
    jspace::Vector pos_;
    jspace::Vector t_;
  };
}
#endif //WBC_FRI_VEL_EST_H
