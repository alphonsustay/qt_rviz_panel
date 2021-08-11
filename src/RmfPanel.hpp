/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__RMFPANEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__RMFPANEL_HPP

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_default_plugins/tools/point/point_tool.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QListView>
#include <QPushButton>
#include <QSpinBox>
#include <QStringListModel>
#include <QTextEdit>
#include <QTimeEdit>
#include <QTimer>
#include <QLabel>
#include <QFont>

#include <memory>
#include <mutex>
#include <thread>

#include "ParseGraph.hpp"

namespace rmf_visualization_rviz2_plugins {

class RmfPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  RmfPanel(QWidget* parent = 0);
  ~RmfPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void send_task_request();
  void cancel_task_request();
  // void queue_loop();
  // void pop_loop();

protected Q_SLOTS:
  void update_fleet_selector();
  // void update_start_waypoint_selector();
  void update_end_waypoint_selector();
  // void update_time_selector();
  // void update_task_summary_list();
  // void update_ongoing_task_list();

private:
  void create_layout();
  // void initialize_publishers();
  void initialize_subscribers();
  void initialize_state_record();
  void initialize_qt_connections();
  // void initialize_models(); 

  // Options - For configuring certain behaviors in the GUI
  // QCheckBox* _simple_mode_checkbox;
  // QCheckBox* _update_time_checkbox; // If checked, update time in _time_selector
  QCheckBox* _workcells_only_checkbox; // If checked, only, workcell waypoints
                                       // will be available for selection
  // Labels for the various fields
  QLabel* _fleet_name_label;
  QLabel* _end_waypoint_name_label;

  
  // Selectors - For targeting agents to accomplish goals
  QComboBox* _fleet_selector;
  // QComboBox* _start_waypoint_selector;
  QComboBox* _end_waypoint_selector;
  // QTimeEdit* _time_selector;

  // QComboBox* _task_id_selector;

  // Status - For visualizing important inforation on the selected agent
  QListView* _fleet_summary_view; // Displays task summaries from rmf_core
  QStringListModel* _fleet_summary_model;
  QStringList _fleet_summary_data;


  // Actions - For queuing commands in Plan
  // QSpinBox* _repeat_count_selector; // Number of loops in Loop task
  QPushButton* _send_loop_button;
  // QPushButton* _cancel_task_button;

  // QTimer to update fields
  QTimer* _update_timer;

  bool _has_loaded = false;

  std::thread _thread;
  std::mutex _mutex;
  rclcpp::Node::SharedPtr _node;

  // ROS2 Plumbing
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    _fleet_state_sub;
  // rclcpp::Subscription<rmf_task_msgs::msg::TaskSummary>::SharedPtr
  //   _task_summary_sub;
  rclcpp::Client<rmf_task_msgs::srv::SubmitTask>::SharedPtr _loop_client;
  // rclcpp::Client<rmf_task_msgs::srv::CancelTask>::SharedPtr _cancel_client;

  // Book Keeping
  std::unordered_map<std::string, std::vector<std::string>>
  _map_fleet_to_robots;
  std::unordered_map<std::string, GraphInfo> _map_fleet_to_graph_info;
  std::unordered_map<std::string,
    rmf_fleet_msgs::msg::RobotState> _map_robot_to_state;
  // std::unordered_map<std::string, std::string> _ongoing_task;

  // Misc Functions
  rmf_utils::optional<GraphInfo>
  load_fleet_graph_info(std::string fleet_name) const;
  // std::string generate_task_uuid(const int len);
  bool waypoint_has_workcell(const std::string waypoint_name,
    const GraphInfo& graph_info);

  // ROS2 callbacks
  void _fleet_state_callback(
    const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  // void _task_summary_callback(
  //   const rmf_task_msgs::msg::TaskSummary::SharedPtr msg);
};
} // namespace rmf_visualization_rviz2_plugins

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__RMFPANEL_HPP
