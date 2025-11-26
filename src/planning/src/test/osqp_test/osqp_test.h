#ifndef OSQP_TEST_H
#define OSQP_TEST_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace Planning
{

  class OsqpTest : public rclcpp::Node
  {
    public:
      OsqpTest();
      void test_Problem();
  };




} // namespace osqp_test
#endif // OSQP_TEST_H