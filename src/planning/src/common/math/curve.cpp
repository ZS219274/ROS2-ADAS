#include "curve.h"

namespace Planning
{
    double Curve::NormalizedAngle(const double &angle)
    {
        double a = std::fmod(angle + M_PI, 2 * M_PI);
        if (a < 0.0)
        {
            a += (2 * M_PI);
        }
        return a - M_PI;
    }

    void Curve::cartesian_to_frenet(const Cartesian &cartesian, const Referential &ref, Frenet &frenet)
    {
        // 计算s
        frenet.s = ref.rs;
        // 计算l
        const double dx = cartesian.x - ref.rx;
        const double dy = cartesian.y - ref.ry;
        const double cos_theta_r = std::cos(ref.rtheta);
        const double sin_theta_r = std::sin(ref.rtheta);
        const double cross_rx_x = cos_theta_r * dy - sin_theta_r * dx;
        frenet.l = std::copysign(std::hypot(dx, dy), cross_rx_x);
        // 计算l' = dl/ds
        const double delta_theta = cartesian.theta - ref.rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);
        const double sin_delta_theta = std::sin(delta_theta);
        const double one_minus_kappa_l = 1 - ref.rkappa * frenet.l;
        frenet.dl_ds = one_minus_kappa_l * tan_delta_theta;
        // 计算l'' = d(dl)/ds
        const double kappa_l_prime = ref.rkappa * frenet.l + ref.rkappa * frenet.dl_ds;
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * ref.rkappa;
        frenet.ddl_ds = -kappa_l_prime * tan_delta_theta +
                        one_minus_kappa_l / (cos_delta_theta * cos_delta_theta) * delta_theta_prime;
        // 计算ds/dt
        frenet.dds_dt = cartesian.speed * cos_delta_theta / one_minus_kappa_l;
        // 计算d(ds)/dt
        frenet.dds_dt = (cartesian.a * cos_delta_theta -
                         (frenet.ds_dt * frenet.ds_dt) * (frenet.dl_ds * delta_theta_prime - kappa_l_prime)) /
                        one_minus_kappa_l;
        // 计算dl/dt
        frenet.dl_dt = cartesian.speed * sin_delta_theta;
        // 计算ddl/dt
        frenet.ddl_dt = cartesian.a * sin_delta_theta;
    }

    void Curve::frenet_to_cartesian(const Frenet &frenet, const Referential &ref, Cartesian &cartesian)
    {
        // 判断 s和rs是否足够近
        if (std::fabs(frenet.s - ref.rs) > delta_s_min)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "reference point s and projection rs don't match s: %.2f, rs: %.2f", frenet.s, ref.rs);
            return;
        }
        // 计算 x和y
        const double cos_theta_r = std::cos(ref.rtheta);
        const double sin_theta_r = std::sin(ref.rtheta);
        cartesian.x = ref.rx - sin_theta_r * frenet.l;
        cartesian.y = ref.ry + cos_theta_r * frenet.l;
        // 计算theta
        const double one_minus_kappa_l = 1 - ref.rkappa * frenet.l;
        const double tan_delta_theta = frenet.dl_ds / one_minus_kappa_l;
        const double delta_theta = std::atan2(frenet.dl_ds, one_minus_kappa_l);
        const double cos_delta_theta = std::cos(delta_theta);
        cartesian.theta = NormalizedAngle(ref.rtheta + delta_theta);
        // 计算kappa
        const double kappa_l_prime = ref.rdkappa * frenet.l + ref.rkappa * frenet.dl_ds;
        cartesian.kappa = ((frenet.ddl_ds + kappa_l_prime * tan_delta_theta) * (cos_delta_theta * cos_delta_theta) / one_minus_kappa_l + ref.rkappa) *
                          cos_delta_theta / one_minus_kappa_l;
        // 计算speed
        cartesian.speed = std::hypot(frenet.ds_dt * one_minus_kappa_l, frenet.ds_dt * frenet.dl_ds);

        // 计算a
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * cartesian.kappa - ref.rkappa;
        cartesian.a = frenet.dds_dt * one_minus_kappa_l / cos_delta_theta + 
            (frenet.ds_dt * frenet.ds_dt) / cos_delta_theta *
            (frenet.dl_ds * delta_theta_prime - kappa_l_prime);
    }

    int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
    {
        const int path_size = path.poses.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_dis = std::numeric_limits<double>::max();
        int closest_point_index = -1;
        for (int i = 0; i < path_size; ++i)
        {
            double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                                    path.poses[i].pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                // 如果上一帧匹配点与当前匹配点间隔大于100，则不匹配
                if (std::abs(last_match_point_index - i) > 100)
                {
                    continue;
                }
                min_dis = dis;
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
    {
        const int path_size = refer_line.refer_line.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_dis = std::numeric_limits<double>::max();
        int closest_point_index = -1;
        for (int i = 0; i < path_size; ++i)
        {
            double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x,
                                    refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                min_dis = dis;
                closest_point_index = i;
            }
        }
        return closest_point_index;
    }

    void Curve::find_projection_point(const Referline &refer_line, const PoseStamped &target_point, Referential &ref)
    {
        // 简化：用匹配点近似替代，前提，参考线足够密且平滑
        const int max_index = find_match_point(refer_line, target_point);
        if (max_index < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "find projection point failed, max_index < 0");
            return;
        }
        ref.rx = refer_line.refer_line[max_index].pose.pose.position.x;
        ref.ry = refer_line.refer_line[max_index].pose.pose.position.y;
        ref.rs = refer_line.refer_line[max_index].rs;
        ref.rtheta = refer_line.refer_line[max_index].rtheta;
        ref.rkappa = refer_line.refer_line[max_index].rkappa;
        ref.rdkappa = refer_line.refer_line[max_index].rdkappa;
    }

    // 计算投影点参数
    void Curve::cal_projection_param(Referline &refer_line)
    {
        const int path_size = refer_line.refer_line.size();
        if (path_size < 3)
        {
            RCLCPP_INFO(rclcpp::get_logger("math"), "refer_line too short, path_size < 3");
            return;
        }

        // 计算rs(累积弧长)
        double rs_tmp = 0.0;
        for (int i = 1; i < path_size; i++)
        {
            if (i == 1)
            {
                rs_tmp = 0.0;
            }
            else
            {
                // 累加每两个点之间的距离
                rs_tmp += std::hypot(refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x,
                                     refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y);
            }
            refer_line.refer_line[i].rs = rs_tmp;
        }

        // 计算航向角
        for (int i = 1; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                refer_line.refer_line[i].rtheta = std::atan2(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                                             refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
            }
            else
            {
                refer_line.refer_line[i].rtheta = std::atan2(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                                             refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
        }

        // 计算曲率
        for (int i = 1; i < path_size - 1; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x,
                                              refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis;
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x,
                                              refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis;
                }
            }
        }
        // 计算曲率变化率
        for (int i = 1; i < path_size - 1; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x,
                                              refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis;
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x,
                                              refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis;
                }
            }
        }
    }
} // namespace Planning
