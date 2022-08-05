
#include "visualization.h"

namespace shenlan {

Visualization::Visualization() : initialized_(false) {}

Visualization::Visualization(ros::NodeHandle &nh) : initialized_(false) {
  initialize(nh);
}

void Visualization::initialize(ros::NodeHandle &nh) {
  if (initialized_) {
    ROS_WARN("Visualization already initialized. Reinitalizing...");
  }

  // register topics
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 10000);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 10000);
  local_trajectory_pub_ =
      nh.advertise<geometry_msgs::PoseArray>("local_trajectory", 10000);

  ref_line_pub_ = nh.advertise<nav_msgs::Path>("reference_line", 10000);

  obstacles_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("obstalces_markers", 1000);

  candidate_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traj_list", 1000);

  initialized_ = true;
}

void Visualization::publishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped> &global_plan) const {
  // if (printErrorWhenNotInitialized()) return;
  base_local_planner::publishPlan(global_plan, global_plan_pub_);
}

void Visualization::publishLocalPlan(
    const std::vector<geometry_msgs::PoseStamped> &local_plan) const {
  // if (printErrorWhenNotInitialized()) return;

  base_local_planner::publishPlan(local_plan, local_plan_pub_);
}

void Visualization::publishGlobalPlan(const nav_msgs::Path &global_plan) {
  // if (printErrorWhenNotInitialized()) return;
  global_plan_pub_.publish(global_plan);
}

void Visualization::publishLocalPlan(const nav_msgs::Path &final_path) {
  ROS_INFO("publishGlobalPlan");
  local_plan_pub_.publish(final_path);
}

void Visualization::publishLocalPlan(const FrenetPath &final_path) {
  nav_msgs::Path local_path;
  local_path.header.frame_id = "gps";
  local_path.header.stamp = ros::Time::now();

  const int size = final_path.t.size();
  for (int i = 0; i < size; i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "gps";
    pose.header.stamp = ros::Time().fromSec(final_path.t[i]);
    pose.pose.position.x = final_path.x[i];
    pose.pose.position.y = final_path.y[i];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(final_path.yaw[i]);
    local_path.poses.push_back(pose);
  }
  local_plan_pub_.publish(local_path);
}

void Visualization::publishLocalTrajectroy(const FrenetPath &final_path) {
  if (printErrorWhenNotInitialized()) return;

  // create pose_array (along trajectory)
  geometry_msgs::PoseArray local_trajectory;
  local_trajectory.header.frame_id = "gps";
  local_trajectory.header.stamp = ros::Time::now();

  const int size = final_path.t.size();
  for (int i = 0; i < size; i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "gps";
    pose.header.stamp = ros::Time().fromSec(final_path.t[i]);
    pose.pose.position.x = final_path.x[i];
    pose.pose.position.y = final_path.y[i];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(final_path.yaw[i]);
    local_trajectory.poses.push_back(pose.pose);
  }
  local_trajectory_pub_.publish(local_trajectory);
}

void Visualization::publishAllLocalTrajectory(const std::vector<FrenetPath>& path_list) {
  if (printErrorWhenNotInitialized()) return;
  double _resolution = 0.2;
  visualization_msgs::MarkerArray LineArray;
  visualization_msgs::Marker Line;

  Line.header.frame_id = "gps";
  Line.header.stamp = ros::Time::now();
  Line.ns = "demo_node/TraLibrary";
  Line.action = visualization_msgs::Marker::ADD;
  Line.pose.orientation.w = 1.0;
  Line.type = visualization_msgs::Marker::LINE_STRIP;
  Line.scale.x = _resolution / 5;

  Line.color.r = 0.0;
  Line.color.g = 0.0;
  Line.color.b = 1.0;
  Line.color.a = 1.0;

  int marker_id = 0;
  const int nums = path_list.size();
  
  for (int i = 0; i < nums; ++i) {
    FrenetPath fp = path_list[i];
    int size = fp.t.size();
    Line.points.clear();
    geometry_msgs::Point pt;
    Line.id = marker_id;
    for (int j = 0; j < size; ++j) {
      pt.x = fp.x[j];
      pt.y = fp.y[j];
      pt.z = 0.0;
      Line.points.push_back(pt);
    }
    LineArray.markers.push_back(Line);
    candidate_traj_pub_.publish(LineArray);
    ++marker_id;
  }
}

void Visualization::publishReferenceLine(const ReferenceLineInfo &ref_line) {
  nav_msgs::Path path;
  path.header.frame_id = "gps";
  path.header.stamp = ros::Time::now();

  const int size = ref_line.size();
  std::cout << "ref line size= " << size;
  for (int i = 0; i < size; i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "gps";
    pose.header.stamp = ros::Time().now();
    pose.pose.position.x = ref_line[i].x;
    pose.pose.position.y = ref_line[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(ref_line[i].hdg);
    path.poses.push_back(pose);
  }
  ref_line_pub_.publish(path);
}

void Visualization::publishObstacles(const std::vector<Poi_f> &obstacles) {
  if (obstacles.empty()) {
    return;
  }

  // Visualize circular obstacles
  std::size_t idx = 0;
  for (const auto &obs : obstacles) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "gps";
    marker.header.stamp = ros::Time::now();
    marker.ns = "CircularObstacles";
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);
    geometry_msgs::Point point;
    point.x = obs[0];
    point.y = obs[1];
    point.z = 0;
    marker.points.push_back(point);

    const double radius = 1.5;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    obstacles_marker_pub_.publish(marker);
  }
}

std_msgs::ColorRGBA Visualization::toColorMsg(double a, double r, double g,
                                              double b) {
  std_msgs::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool Visualization::printErrorWhenNotInitialized() {
  if (!initialized_) {
    ROS_ERROR(
        "Visualization class not initialized. You must call initialize or "
        "an appropriate constructor");
    return true;
  }
  return false;
}

}  // namespace shenlan
