#ifndef DROP_LAYER_H_
#define DROP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include <xbot_msgs/InfraRed.h>

// #include <kobuki_msgs/CliffEvent.h>

namespace drop_layer_namespace
{

class DropLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  DropLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  bool isDiscretized(){
    return true;
  }

  void DropLayerCallback(const xbot_msgs::InfraRed &msg);

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  bool need_mark1;
  bool need_mark2;
  double range_sensor;
  double mark_x_, mark_y_;

  ros::Subscriber cliff_sub;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
