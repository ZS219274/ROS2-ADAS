#ifndef OSQP_TEST_H
#define OSQP_TEST_H

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

namespace Planning {

class OsqpTest : public rclcpp::Node {
 public:
  OsqpTest();
  void test_Problem();
};

}  // namespace Planning
#endif  // OSQP_TEST_H