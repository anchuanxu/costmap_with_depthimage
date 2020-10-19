#include <drop_layers/drop_layers.h>
#include <pluginlib/class_list_macros.h>
#include <xbot_msgs/InfraRed.h>

PLUGINLIB_EXPORT_CLASS(drop_layer_namespace::DropLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace drop_layer_namespace
{

DropLayer::DropLayer() {}

void DropLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  std::string topics_string;
  nh.param("range", range_sensor, 0.1);
  cliff_sub = nh.subscribe("/mobile_base/sensors/infrared", 10, &DropLayer::DropLayerCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &DropLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DropLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void DropLayer::DropLayerCallback(const xbot_msgs::InfraRed &msg)
{
//  此处留有接口，返回底部防跌落红外传感器的情况
//  ROS_INFO("need_mark1 = %d", msg.front_hanged);
//  ROS_INFO("need_mark2 = %d", msg.rear_hanged);
  if(msg.front_hanged == true && msg.rear_hanged == true)
  {
    //前后都是悬崖
    need_mark1 = true;
    need_mark2 = true;
  }
  else if(msg.front_hanged == true && msg.rear_hanged == false)
  {
    //前方是悬崖
    need_mark1 = true;
    need_mark2 = false;
  }
  else if(msg.front_hanged == false && msg.rear_hanged == true)
  {
    //后方是悬崖
    need_mark1 = false;
    need_mark2 = true;
  }
  else {
    //没有悬崖
    need_mark1 = false;
    need_mark2 = false;
  }
}

void DropLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void DropLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if(need_mark1)
  {
    double mark_forward = 1;
    double mark_x = robot_x + mark_forward * range_sensor * cos(robot_yaw);
    double mark_y = robot_y + mark_forward * range_sensor * sin(robot_yaw);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my))
      setCost(mx, my, LETHAL_OBSTACLE);
    
    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
  }
  if(need_mark2)
  {
    double mark_forward = -1;
    double mark_x = robot_x + mark_forward * range_sensor * cos(robot_yaw);
    double mark_y = robot_y + mark_forward * range_sensor * sin(robot_yaw);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my))
      setCost(mx, my, LETHAL_OBSTACLE);

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
  }
  
}

void DropLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

} // end namespace
