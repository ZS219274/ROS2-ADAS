#include "local_path_planner.h"

namespace Planning
{
  LocalPathPlanner::LocalPathPlanner() // 局部路径规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("LocalPathPlanner"), "LocalPathPlanner created!");
    local_path_config_ = std::make_unique<ConfigReader>();
    local_path_config_->read_local_path_config();
    local_path_smoother_ = std::make_shared<LocalPathSmoother>();
  }

  LocalPath LocalPathPlanner::create_local_path(const Referline &refer_line,
                                                const std::shared_ptr<VehicleInfoBase> &car,
                                                const std::shared_ptr<DecisionCenter> &decision)
  {
    // 初始化局部路径
    init_local_path();

    // 计算路径点的sl值
    double point_s = car->s(); // 临时点s, 先记录车辆当前的s值
    LocalPathPoint point_tmp;
    for (int i = 0; i < local_path_config_->local_path().path_size_; i++)
    {
      // 规划的起点
      point_s += car->ds_dt();                       // 路径规划的起点：下一帧参考线投影点的s //速度单位m/帧， 不是m/s
      if (point_s > refer_line.refer_line.back().rs) // 超出参考线末端，则退出
      {
        break;
      }

      // 给point_tmp赋值， 先把s和ds_dt赋上，l和dl_ds初始化
      point_tmp.s = point_s;
      point_tmp.ds_dt = car->ds_dt();
      point_tmp.dds_dt = car->dds_dt();
      point_tmp.l = 0.0;
      point_tmp.dl_ds = 0.0;
      point_tmp.ddl_ds = 0.0;

      // 计算point_tmp的l和dl_ds值
      const int sl_points_size = decision->sl_points().size();
      for (int j = 0; j < sl_points_size; j++)
      {
        // 确定每个分段的起始状态和末状态
        const double start_s = decision->sl_points()[j].s_;
        const double start_l = decision->sl_points()[j].l_;
        const double start_dl_ds = 0.0;
        const double start_ddl_ds = 0.0;
        const double end_s = decision->sl_points()[j + 1].s_;
        const double end_l = decision->sl_points()[j + 1].l_;
        const double end_dl_ds = 0.0;
        const double end_ddl_ds = 0.0;

        // 如果临时点的s在分段范围内
        if (point_s >= start_s && point_s < end_s)
        {
          const double point_s2 = point_s * point_s;
          const double point_s3 = point_s2 * point_s;
          const double point_s4 = point_s3 * point_s;
          const double point_s5 = point_s4 * point_s;
          const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(start_s, start_l, start_dl_ds,
                                                                                 start_ddl_ds, end_s, end_l,
                                                                                 end_dl_ds, end_ddl_ds);
          point_tmp.l = a(0) + a(1) * point_s + a(2) * point_s2 + a(3) * point_s3 + a(4) * point_s4 + a(5) * point_s5;
          point_tmp.dl_ds = a(1) + 2.0 * a(2) * point_s + 3.0 * a(3) * point_s2 + 4.0 * a(4) * point_s3 + 5.0 * a(5) * point_s4;
          point_tmp.ddl_ds = 2.0 * a(2) + 6.0 * a(3) * point_s + 12.0 * a(4) * point_s2 + 20.0 * a(5) * point_s3;
        }
      }
      local_path_.local_path.emplace_back(point_tmp); // 临时点加入路径
    }

    // sl坐标下平滑
    local_path_smoother_->smooth_local_path(local_path_);

    // 转笛卡尔
    tf2::Quaternion qtn;
    for (auto &point : local_path_.local_path)
    {
      // 计算路径点在参考线上的投影
      const double rs = point.s;
      const int match_index = Curve::find_match_point(refer_line, rs);
      const double rx = refer_line.refer_line[match_index].pose.pose.position.x;
      const double ry = refer_line.refer_line[match_index].pose.pose.position.y;
      const double rtheta = refer_line.refer_line[match_index].rtheta;
      const double rkappa = refer_line.refer_line[match_index].rkappa;
      const double rdkappa = refer_line.refer_line[match_index].rdkappa;

      // 计算路径点在笛卡尔坐标下参数
      double x, y, theta, kappa, speed, a;
      Frenet frenet_tmp
      {
        .s = point.s,
        .ds_dt = point.ds_dt,
        .dds_dt = point.dds_dt,
        .l = point.l,
        .dl_ds = point.dl_ds,
        .ddl_ds = point.ddl_ds
      };
      Referential ref_tmp 
      {
        .rs = rs,
        .rx = rx,
        .ry = ry,
        .rtheta = rtheta,
        .rkappa = rkappa,
        .rdkappa = rdkappa
      };
      Cartesian cartesian_tmp;
      // frenet_tmp.s = point.s;
      // frenet_tmp.ds_dt = point.ds_dt;
      // frenet_tmp.dds_dt = point.dds_dt;
      // frenet_tmp.l = point.l;
      // frenet_tmp.dl_ds = point.dl_ds;
      // frenet_tmp.ddl_ds = point.ddl_ds;

      // ref_tmp.rs = rs;
      // ref_tmp.rx = rx;
      // ref_tmp.ry = ry;
      // ref_tmp.rtheta = rtheta;
      // ref_tmp.rkappa = rkappa;
      // ref_tmp.rdkappa = rdkappa;
      Curve::frenet_to_cartesian(frenet_tmp, ref_tmp, cartesian_tmp);
      point.pose.header = local_path_.header;
      point.pose.pose.position.x = cartesian_tmp.x;
      point.pose.pose.position.y = cartesian_tmp.y;
      point.rtheta = cartesian_tmp.theta;
      point.kappa = cartesian_tmp.kappa;

      qtn.setRPY(0.0, 0.0, cartesian_tmp.theta);
      point.pose.pose.orientation.x = qtn.x();
      point.pose.pose.orientation.y = qtn.y();
      point.pose.pose.orientation.z = qtn.z();
      point.pose.pose.orientation.w = qtn.w();
    }

    // 计算投影点参数
    Curve::cal_projection_param(local_path_);
    RCLCPP_INFO(rclcpp::get_logger("local_path"), "local path size: %d", local_path_.local_path.size());
    return local_path_;
  }

  Path LocalPathPlanner::path_to_rviz()
  {
    local_path_rviz_.header = local_path_.header;
    local_path_rviz_.poses.clear();

    PoseStamped point_tmp;
    point_tmp.header = local_path_rviz_.header;
    for (const auto &point : local_path_.local_path)
    {
      point_tmp.pose = point.pose.pose;
      local_path_rviz_.poses.emplace_back(point_tmp);
    }
    return local_path_rviz_;
  }

  void LocalPathPlanner::init_local_path()
  {
    local_path_.header.frame_id = local_path_config_->pnc_map().frame_; // 坐标
    local_path_.header.stamp = rclcpp::Clock().now();                   // 每一帧都更新时间
    local_path_.local_path.clear();                                     // 每一帧都清空，从新规划
  }
} // namespace Planning