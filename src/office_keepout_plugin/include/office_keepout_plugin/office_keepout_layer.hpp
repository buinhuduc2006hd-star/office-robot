#ifndef OFFICE_KEEPOUT_PLUGIN__OFFICE_KEEPOUT_LAYER_HPP_
#define OFFICE_KEEPOUT_PLUGIN__OFFICE_KEEPOUT_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <office_interfaces/msg/active_keepout_zones.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <vector>
#include <string>

namespace office_keepout_plugin
{

struct BoundingBox {
  double min_x, min_y, max_x, max_y;
};

// Polygon đã transform sang frame của costmap
struct TransformedZone {
  std::string room_id;
  std::string door_id;
  geometry_msgs::msg::Polygon polygon;  // tọa độ trong costmap frame
  BoundingBox bbox;
};

class OfficeKeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  OfficeKeepoutLayer();
  virtual ~OfficeKeepoutLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;
  virtual bool isClearable() override {return false;}

private:
  void activeZonesCallback(const office_interfaces::msg::ActiveKeepoutZones::SharedPtr msg);
  BoundingBox computePolygonBounds(const geometry_msgs::msg::Polygon & poly);
  bool isPointInPolygon(double x, double y, const geometry_msgs::msg::Polygon & poly);

  // Transform polygon từ source_frame sang target_frame
  bool transformPolygon(
    const geometry_msgs::msg::Polygon & in_poly,
    const std::string & source_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Polygon & out_poly);

  rclcpp::Subscription<office_interfaces::msg::ActiveKeepoutZones>::SharedPtr active_zones_sub_;

  // Raw zones từ message (trong map frame)
  std::vector<office_interfaces::msg::KeepoutZone> raw_zones_;
  std::string zones_frame_;   // frame_id của zones (thường là "map")

  // Zones đã transform sang costmap frame, sẵn sàng dùng trong updateCosts
  std::vector<TransformedZone> transformed_zones_;
  std::vector<BoundingBox> dirty_bounds_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  bool need_update_;
  bool zones_changed_;  // có raw_zones_ mới cần transform không
};

}  // namespace office_keepout_plugin

#endif  // OFFICE_KEEPOUT_PLUGIN__OFFICE_KEEPOUT_LAYER_HPP_
