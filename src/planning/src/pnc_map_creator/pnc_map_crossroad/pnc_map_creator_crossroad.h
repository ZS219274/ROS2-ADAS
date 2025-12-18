#ifndef PNC_MAP_CREATOR_CROSSROAD_H_
#define PNC_MAP_CREATOR_CROSSROAD_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
  class PncMapCreatorCrossroad : public PncMapCreatorBase // 交叉路口地图
  {
  public:
    PncMapCreatorCrossroad();
    PNCMap create_pnc_map() override;

  private:
    void init_pnc_map();                                                                            // 初始化pnc地图
    void draw_straight_x(const double &length, const double &plus_flag, const double &ratio = 1.0); // 画直道x
    void draw_straight_y(const double &length, const double &plus_flag, const double &ratio = 1.0); // 画直道y
    void draw_arc_x(const double &angle, const double &plus_flag, const double &ratio = 1.0);
    void draw_arc_y(const double &angle, const double &plus_flag, const double &ratio = 1.0);

    // 新增交叉路口专用方法
    void draw_crossroad_t(Point &s_edge_point, Point &n_edge_point, Point &e_edge_point);     // 绘制十字型交叉路口
  };
} // namespace Planning

#endif // PNC_MAP_CREATOR_CROSSROAD_H_