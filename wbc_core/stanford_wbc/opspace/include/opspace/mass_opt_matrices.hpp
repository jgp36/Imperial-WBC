
#ifndef OPSPACE_MASS_OPT_HPP
#define OPSPACE_MASS_OPT_HPP
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Geometry>
#include <jspace/State.hpp>

using namespace jspace;

namespace opspace {

class mass_opt {

public:
  static Eigen::Matrix<double,7,7> d1A(Vector position);
  static Eigen::Matrix<double,7,7> d2A(Vector position);
  static Eigen::Matrix<double,7,7> d3A(Vector position);
  static Eigen::Matrix<double,7,7> d4A(Vector position);
  static Eigen::Matrix<double,7,7> d5A(Vector position);
  static Eigen::Matrix<double,7,7> d6A(Vector position);
  static Eigen::Matrix<double,7,7> d7A(Vector position);

  static Eigen::Matrix<double,3,7> d1Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d2Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d3Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d4Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d5Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d6Jv(Vector control_point, Vector position);
  static Eigen::Matrix<double,3,7> d7Jv(Vector control_point, Vector position);

  static Eigen::Matrix<double,3,7> d1Jw(Vector position);
  static Eigen::Matrix<double,3,7> d2Jw(Vector position);
  static Eigen::Matrix<double,3,7> d3Jw(Vector position);
  static Eigen::Matrix<double,3,7> d4Jw(Vector position);
  static Eigen::Matrix<double,3,7> d5Jw(Vector position);
  static Eigen::Matrix<double,3,7> d6Jw(Vector position);
  static Eigen::Matrix<double,3,7> d7Jw(Vector position);
private:
  static Eigen::Matrix<double,7,7> d2A2(Vector position);
  static Eigen::Matrix<double,7,7> d2A3(Vector position);
  static Eigen::Matrix<double,7,7> d2A4(Vector position);
  static Eigen::Matrix<double,7,7> d2A5(Vector position);
  static Eigen::Matrix<double,7,7> d2A6(Vector position);
  static Eigen::Matrix<double,7,7> d2A7(Vector position);

  static Eigen::Matrix<double,7,7> d3A3(Vector position);
  static Eigen::Matrix<double,7,7> d3A4(Vector position);
  static Eigen::Matrix<double,7,7> d3A5(Vector position);
  static Eigen::Matrix<double,7,7> d3A6(Vector position);
  static Eigen::Matrix<double,7,7> d3A7(Vector position);

  static Eigen::Matrix<double,7,7> d4A4(Vector position);
  static Eigen::Matrix<double,7,7> d4A5(Vector position);
  static Eigen::Matrix<double,7,7> d4A6(Vector position);
  static Eigen::Matrix<double,7,7> d4A7(Vector position);

  static Eigen::Matrix<double,7,7> d5A5(Vector position);
  static Eigen::Matrix<double,7,7> d5A6(Vector position);
  static Eigen::Matrix<double,7,7> d5A7(Vector position);

  static Eigen::Matrix<double,7,7> d6A6(Vector position);
  static Eigen::Matrix<double,7,7> d6A7(Vector position);

  static Eigen::Matrix<double,7,7> d7A7(Vector position);

};

}

#endif
