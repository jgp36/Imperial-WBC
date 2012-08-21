#include <wbc_fri/vel_est.h>

using namespace wbc_fri;
using namespace jspace;

void vel_est::
init(double R, double rj, int m, double pos, double t)
{
  R_ = R;
  rj_ = rj;
  m_ = m;
  sj_ = 2/rj;
  pos_ = pos*Vector::Ones(m); //inefficient but easier
  t_ = t*Vector::Ones(m);
}

double vel_est::
update(double pos, double t) {
  double vel;
  //Calculate velocity
  for (size_t ii(0); ii < m_; ++ii) {
    if (abs(pos-pos_[ii]) > sj_*R_ || ii == m_-1) {
      vel = (pos-pos_[ii])/(t-t_[ii]);
      break;
    }
  }

  //Update pos and vel list
  double pprev = pos_[0];
  double tprev = t_[0];
  pos_[0] = pos;
  t_[0] = t;
  for (size_t ii(1); ii<m_; ++ii) {
    double ptemp = pos_[ii];
    double ttemp = t_[ii];
    pos_[ii] = pprev;
    t_[ii] = tprev;
    pprev = ptemp;
    tprev = ttemp;
  }

  vel = (pos - pos_[1])/(t-t_[1]);
  return vel;
  
}
