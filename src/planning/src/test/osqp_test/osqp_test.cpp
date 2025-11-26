#include "osqp_test.h"

namespace Planning
{
  OsqpTest::OsqpTest() : Node("osqp_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "OsqpTest node created.");
    // Run test
    test_Problem();
  }

  void OsqpTest::test_Problem()
  {
    Eigen::SparseMatrix<double> P(2, 2); // 2x2矩阵
    Eigen::VectorXd Q(2);                // 2x1向量
    Eigen::SparseMatrix<double> A(2, 2); // 2x2单位矩阵
    Eigen::VectorXd LowerBound(2);       // 2x1向量  上边界
    Eigen::VectorXd UpperBound(2);       // 2x1向量   下边界

    P.insert(0, 0) = 2.0;                // 矩阵赋值
    P.insert(1, 1) = 2.0;                // 矩阵赋值 为赋值默认0
    std::cout << "P:" << std::endl
              << P << std::endl;
    Q << -2, -2;                         // 列向量赋值
    std::cout << "Q:" << std::endl
              << Q << std::endl;
    A.insert(0, 0) = 1.0;                // 单位矩阵赋值
    A.insert(1, 1) = 1.0;
    std::cout << "A:" << std::endl
              << A << std::endl;
    LowerBound << 0.0, 0.0;              // 下边界赋值
    UpperBound << 1.5, 1.5;              // 上边界赋值

    // 创建求解器
    OsqpEigen::Solver solver;

    // 设置求解器参数
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // 初始化(数据填充到data)
    solver.data()->setNumberOfVariables(2);   // 2个未知数
    solver.data()->setNumberOfConstraints(2);  // 2个约束条件 
    
    if(!solver.data()->setHessianMatrix(P))
    {
      return;
    }
    if(!solver.data()->setGradient(Q))
    {
      return;
    }
    if(!solver.data()->setLinearConstraintsMatrix(A))
    {
      return;
    }
    if(!solver.data()->setLowerBound(LowerBound))
    {
      return;
    }
    if(!solver.data()->setUpperBound(UpperBound))
    {
      return;
    }
    if(!solver.initSolver())
    {
      return;
    }
    /*初始化完成*/

    // 求解 1.最小值 2.最小值时 x1 x2的值
    // 待求解的值
    Eigen::VectorXd QPsolution;
    
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      return;
    }
    QPsolution = solver.getSolution();
    std::cout << "QPsolution:" << std::endl
              << QPsolution << std::endl;

  }

} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planning::OsqpTest>());
  rclcpp::shutdown();
  return 0;
}