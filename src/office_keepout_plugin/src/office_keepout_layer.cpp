#include "office_keepout_plugin/office_keepout_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(office_keepout_plugin::OfficeKeepoutLayer, nav2_costmap_2d::Layer)

namespace office_keepout_plugin
{

OfficeKeepoutLayer::OfficeKeepoutLayer()
: need_update_(false), zones_changed_(false)
{
}

OfficeKeepoutLayer::~OfficeKeepoutLayer() {}

void OfficeKeepoutLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Khởi tạo TF buffer và listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  active_zones_sub_ = node->create_subscription<office_interfaces::msg::ActiveKeepoutZones>(
    "/office/keepout_updates", 10,
    std::bind(&OfficeKeepoutLayer::activeZonesCallback, this, std::placeholders::_1));

  zones_frame_ = "map";
  current_ = true;

  RCLCPP_INFO(node->get_logger(), "OfficeKeepoutLayer initialized on '/office/keepout_updates'.");
}

void OfficeKeepoutLayer::activeZonesCallback(
  const office_interfaces::msg::ActiveKeepoutZones::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Lưu dirty bounds cho các zone CŨ (đã bị xóa)
  for (const auto & old_z : transformed_zones_) {
    bool found = false;
    for (const auto & new_z : msg->zones) {
      if (old_z.room_id == new_z.room_id && old_z.door_id == new_z.door_id) {
        found = true;
        break;
      }
    }
    if (!found) {
      dirty_bounds_.push_back(old_z.bbox);
    }
  }

  // Lưu raw zones mới và frame_id
  raw_zones_ = msg->zones;
  zones_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  zones_changed_ = true;
  need_update_ = true;
}

bool OfficeKeepoutLayer::transformPolygon(
  const geometry_msgs::msg::Polygon & in_poly,
  const std::string & source_frame,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::Polygon & out_poly)
{
  if (source_frame == target_frame) {
    out_poly = in_poly;
    return true;
  }

  try {
    auto transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, stamp,
      rclcpp::Duration::from_seconds(0.2));

    out_poly.points.clear();
    for (const auto & pt : in_poly.points) {
      geometry_msgs::msg::PointStamped ps_in, ps_out;
      ps_in.header.frame_id = source_frame;
      ps_in.header.stamp = stamp;
      ps_in.point.x = pt.x;
      ps_in.point.y = pt.y;
      ps_in.point.z = pt.z;
      tf2::doTransform(ps_in, ps_out, transform);
      geometry_msgs::msg::Point32 out_pt;
      out_pt.x = static_cast<float>(ps_out.point.x);
      out_pt.y = static_cast<float>(ps_out.point.y);
      out_pt.z = 0.0f;
      out_poly.points.push_back(out_pt);
    }
    return true;
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "OfficeKeepoutLayer: TF transform %s→%s failed: %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return false;
  }
}

BoundingBox OfficeKeepoutLayer::computePolygonBounds(
  const geometry_msgs::msg::Polygon & poly)
{
  BoundingBox b = {1e9, 1e9, -1e9, -1e9};
  for (const auto & p : poly.points) {
    b.min_x = std::min(b.min_x, static_cast<double>(p.x));
    b.min_y = std::min(b.min_y, static_cast<double>(p.y));
    b.max_x = std::max(b.max_x, static_cast<double>(p.x));
    b.max_y = std::max(b.max_y, static_cast<double>(p.y));
  }
  // Mở rộng bounds thêm margin để bao phủ toàn bộ resolution của costmap
  double margin = (poly.points.size() == 2) ? 0.25 : 0.15;
  b.min_x -= margin;
  b.min_y -= margin;
  b.max_x += margin;
  b.max_y += margin;
  return b;
}

void OfficeKeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!need_update_ && raw_zones_.empty() && dirty_bounds_.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // LUÔN LUÔN transform polygon mỗi chu kỳ (để bắt kịp sự thay đổi của TF map -> odom khi AMCL cập nhật)
  std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  auto node = node_.lock();
  rclcpp::Time now = node ? node->now() : rclcpp::Time(0);

  // Lưu lại transformed zones của chu kỳ trước vào dirty_bounds_ để xóa đi vùng đã bị lệch
  for (const auto & old_tz : transformed_zones_) {
    dirty_bounds_.push_back(old_tz.bbox);
  }
  
  transformed_zones_.clear();
  for (const auto & raw_z : raw_zones_) {
    geometry_msgs::msg::Polygon transformed_poly;
    bool ok = transformPolygon(raw_z.polygon, zones_frame_, costmap_frame, now, transformed_poly);
    if (!ok) {
      ok = transformPolygon(raw_z.polygon, zones_frame_, costmap_frame, rclcpp::Time(0), transformed_poly);
    }
    if (ok && !transformed_poly.points.empty()) {
      TransformedZone tz;
      tz.room_id = raw_z.room_id;
      tz.door_id = raw_z.door_id;
      tz.polygon = transformed_poly;
      tz.bbox = computePolygonBounds(transformed_poly);
      transformed_zones_.push_back(tz);
      
      // Chỉ log ở mức DEBUG để không làm spam terminal
      if (zones_changed_ && node) {
        RCLCPP_DEBUG(
          node->get_logger(),
          "[Keepout] Zone %s/%s transformed to frame '%s': bbox=[%.2f,%.2f,%.2f,%.2f]",
          raw_z.room_id.c_str(), raw_z.door_id.c_str(), costmap_frame.c_str(),
          tz.bbox.min_x, tz.bbox.min_y, tz.bbox.max_x, tz.bbox.max_y);
      }
    }
  }
  zones_changed_ = false;

  // Mở rộng dirty bounds (vùng cần xóa obstacle cũ)
  for (const auto & b : dirty_bounds_) {
    *min_x = std::min(*min_x, b.min_x);
    *min_y = std::min(*min_y, b.min_y);
    *max_x = std::max(*max_x, b.max_x);
    *max_y = std::max(*max_y, b.max_y);
  }

  // Mở rộng bounds để cover active zones hiện tại
  for (const auto & tz : transformed_zones_) {
    *min_x = std::min(*min_x, tz.bbox.min_x);
    *min_y = std::min(*min_y, tz.bbox.min_y);
    *max_x = std::max(*max_x, tz.bbox.max_x);
    *max_y = std::max(*max_y, tz.bbox.max_y);
  }
}

bool OfficeKeepoutLayer::isPointInPolygon(
  double x, double y, const geometry_msgs::msg::Polygon & poly)
{
  int n = static_cast<int>(poly.points.size());
  if (n < 2) {return false;}

  // Nếu polygon là đoạn thẳng 2 điểm P1-P2 (đường nối giữa 2 điểm của cửa)
  if (n == 2) {
    double x1 = poly.points[0].x, y1 = poly.points[0].y;
    double x2 = poly.points[1].x, y2 = poly.points[1].y;
    double dx = x2 - x1, dy = y2 - y1;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-6) {return false;}
    double t = std::max(0.0, std::min(1.0, ((x - x1) * dx + (y - y1) * dy) / len_sq));
    double proj_x = x1 + t * dx;
    double proj_y = y1 + t * dy;
    double dist_sq = (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y);
    double threshold = 0.20;  // Bán kính cản 20cm (độ dày tường cản 40cm bao quanh vạch cửa)
    return dist_sq <= (threshold * threshold);
  }

  // Nếu polygon >= 3 điểm (thuật toán ray-casting tiêu chuẩn)
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = poly.points[i].x, yi = poly.points[i].y;
    double xj = poly.points[j].x, yj = poly.points[j].y;
    bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) {inside = !inside;}
  }
  return inside;
}

void OfficeKeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Tô LETHAL_OBSTACLE cho tất cả cells trong active zones
  if (!transformed_zones_.empty()) {
    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        for (const auto & tz : transformed_zones_) {
          // Kiểm tra bounding box trước để tăng tốc
          if (wx < tz.bbox.min_x || wx > tz.bbox.max_x ||
            wy < tz.bbox.min_y || wy > tz.bbox.max_y)
          {
            continue;
          }
          if (isPointInPolygon(wx, wy, tz.polygon)) {
            master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
            break;
          }
        }
      }
    }
  }

  // Xóa dirty bounds tracking sau khi updateCosts xong
  dirty_bounds_.clear();

  if (transformed_zones_.empty()) {
    need_update_ = false;
  }
}

void OfficeKeepoutLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  raw_zones_.clear();
  transformed_zones_.clear();
  dirty_bounds_.clear();
  zones_changed_ = false;
  need_update_ = true;
}

}  // namespace office_keepout_plugin
