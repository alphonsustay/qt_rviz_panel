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
#include "RmfPanel.hpp"
#include "StandardNames.hpp"

#include <QGroupBox>
#include <QLabel>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <rmf_traffic/geometry/Circle.hpp>

#include <random>

namespace rmf_visualization_rviz2_plugins {

using FleetState = rmf_fleet_msgs::msg::FleetState;
using RobotState = rmf_fleet_msgs::msg::RobotState;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using TaskSummary = rmf_task_msgs::msg::TaskSummary;
using Graph = rmf_traffic::agv::Graph;
using Bool = std_msgs::msg::Bool;

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel");
  _update_timer = new QTimer(this);
  _update_timer->start(1000); // Update clock running at 1Hz

  create_layout();
  initialize_subscribers();
  initialize_state_record();
  initialize_qt_connections();

  _thread = std::thread([&]() { rclcpp::spin(_node); });

  _has_loaded = true;
}

RmfPanel::~RmfPanel()
{
  if (_has_loaded)
  {
    _thread.join();
    rclcpp::shutdown();
  }
}

//==================================================================================
void RmfPanel::create_layout()
{
  // Creates the layout for QT GUI
  QGridLayout* control_panel_layout = new QGridLayout; 
  // Setting the Layout
  QGroupBox* selector_gb = new QGroupBox("AMR Task Selectors");
  QGridLayout* selector_layout = new QGridLayout();
  QFont _font_format = QFont("Source Code Pro", 20);

  // Fleet Variables
   _fleet_name_label = new QLabel("Fleet:");
  _fleet_name_label->setFont(_font_format);
  
  _fleet_selector = new QComboBox;
  _fleet_selector->setMinimumSize(QSize(50,50));
  _fleet_selector->setFont(_font_format);

  selector_layout->addWidget(_fleet_name_label, 0, 0, 2, 1);
  selector_layout->addWidget(_fleet_selector, 0, 1, 2, 2);

  // End Waypoint Variables
  _end_waypoint_name_label = new QLabel("End Waypoint: ");
  _end_waypoint_name_label->setFont(_font_format);

  _end_waypoint_selector = new QComboBox;
  _end_waypoint_selector->setEditable(true);
  _end_waypoint_selector->setMinimumSize(QSize(50,50));
  _end_waypoint_selector->setFont(_font_format);

  selector_layout->addWidget(_end_waypoint_name_label, 4, 0, 2, 1);
  selector_layout->addWidget(_end_waypoint_selector, 4, 1, 2, 2);

  // Send Task Button
  _send_loop_button = new QPushButton("Send Loop Request");
  _send_loop_button->setFont(_font_format);
  _send_loop_button->setMinimumSize(QSize(100,100));

  selector_layout->addWidget(_send_loop_button, 8, 0, 4, -1);

  selector_gb->setLayout(selector_layout); 

  control_panel_layout->addWidget(selector_gb, 0, 0, 12, 5);
  setLayout(control_panel_layout);
}

// Initialization Functions
//==================================================================================

void RmfPanel::initialize_subscribers()
{
  _fleet_state_sub = _node->create_subscription<FleetState>(
    rmf_visualization_rviz2_plugins::FleetStateTopicName, 10,
    std::bind(&RmfPanel::_fleet_state_callback, this, std::placeholders::_1));

}

//==================================================================================
void RmfPanel::initialize_state_record()
{
  // These data structures allow lookup of states of various important agents
  _map_fleet_to_robots =
    std::unordered_map<std::string, std::vector<std::string>>();
  _map_fleet_to_graph_info = std::unordered_map<std::string, GraphInfo>();
}

//==================================================================================

void RmfPanel::initialize_qt_connections()
{
  connect(this, SIGNAL(configChanged()), this, 
    SLOT(update_fleet_selector()));

  connect(_fleet_selector, SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(update_end_waypoint_selector()));

  connect(_send_loop_button, SIGNAL(clicked()), this,
    SLOT(send_task_request()));
}

// Misc Functions
//==================================================================================
rmf_utils::optional<GraphInfo>
RmfPanel::load_fleet_graph_info(std::string fleet_name) const
{
  // TODO(BH): Currently mocking up VehicleTraits, potential to give more
  // accurate values

  RCLCPP_INFO(_node->get_logger(), "Loading " + fleet_name + "..");

  rclcpp::Node::SharedPtr _param_node =
    std::make_shared<rclcpp::Node>("nav_graph_param_loader");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    _param_node, fleet_name + "_fleet_adapter");

  // Wait for service to be available. After a cutoff duration, we can
  // conclude that the fleet_adapter was named wrongly
  if (!parameters_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Parameter service not found for " + fleet_name + ".");
    RCLCPP_ERROR(
      _node->get_logger(), "Please check that the fleet adapter is named "
      + fleet_name + "_fleet_adapter");
    return rmf_utils::nullopt;
  }

  // Try to load nav graph
  try
  {
    auto nav_graph_path_parameters =
      parameters_client->get_parameters({"nav_graph_file"});
    std::string nav_file_path = nav_graph_path_parameters[0].as_string();
    std::cout << "Nav File Path Found: " + nav_file_path << std::endl;

    const auto footprint = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
    auto traits = rmf_traffic::agv::VehicleTraits{
      {1.0, 1.0},
      {1.0, 1.0},
      {footprint}};

    rmf_utils::optional<GraphInfo> graph_info =
      parse_graph(nav_file_path, traits, *_node);
    return graph_info;
  }

  // If the nav graph is not available, these should help debug.
  catch (rclcpp::ParameterTypeException& e)
  {
    RCLCPP_INFO(_node->get_logger(), "Nav File not found. \n");
    RCLCPP_INFO(_node->get_logger(),
      "If this adapter is Read Only, this is fine. \n");
    RCLCPP_INFO(_node->get_logger(),
      "If this adapter is Full Control, this should not happen. \n");
    RCLCPP_INFO(_node->get_logger(),
      "Check that the launch file parameter 'nav_graph_file' is correct. \n");
    return rmf_utils::nullopt;
  }
}

// Load config
void RmfPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

// Save config
void RmfPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// Q_SLOTS
// Actions
void RmfPanel::update_fire_alarm_state()
{
  if (_fire_alarm_state == true)
  {
    _fire_alarm_state = false;
  }
  else
  {
    _fire_alarm_state = true;  
  }
  return;
}

void RmfPanel::send_task_request()
{
  std::shared_ptr<rclcpp::Node> _loop_task = rclcpp::Node::make_shared("Loop_Client");
  _loop_client = _loop_task->create_client<SubmitTask>(
  rmf_visualization_rviz2_plugins::LoopRequestServiceName);

  while (!_loop_client->wait_for_service(std::chrono::seconds(1))) 
  {
    if (!rclcpp::ok()) 
    {
        RCLCPP_ERROR(_loop_task->get_logger(), "client interrupted while waiting for service to appear.");
        return;
    }
    RCLCPP_INFO(_loop_task->get_logger(), "waiting for service to appear...");
  }

  // std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();

  auto request = std::make_shared<Request>();
  Loop loop;
  TaskType tasktype;
  Priority priority;
  TaskDescription description;
  // If either start or end are empty, the task should probably not be queued
  if (end.empty())//(start.empty() || end.empty())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Waypoint input is empty string; Task not queued in plan.");
    return;
  }

  priority.value = 0;
  tasktype.type = rmf_task_msgs::msg::TaskType().TYPE_LOOP;

  loop.num_loops = 1;
  loop.start_name = end;
  loop.finish_name = end;

  description.priority = priority;
  description.task_type = tasktype;
  description.loop = loop;

  request->description = description;
  request->requester = _loop_task->get_name();


  RCLCPP_INFO(_loop_task->get_logger(), "Submitting Loop Task Request");
  auto result_future = _loop_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(_loop_task, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(_loop_task->get_logger(), "service call failed :");
  }

  /**
   *  Checking service call return response
   */
  auto result = result_future.get();
  if (!(result->success))
  {
    RCLCPP_INFO(_loop_task->get_logger(), result->message); 
    RCLCPP_INFO(_loop_task->get_logger(), result->task_id);
  }
  else
  {
    RCLCPP_INFO(_loop_task->get_logger(), 
      "service call successful, the task ID to watch is '%s'", 
      (result->task_id).c_str());
  }
}

// Updates
void RmfPanel::update_fleet_selector()
{
  bool new_fleet_found =
    (_fleet_selector->count() != (int)_map_fleet_to_robots.size());
  if (new_fleet_found)
  {
    RCLCPP_INFO(_node->get_logger(), "New Fleet found, refreshing...");
    _fleet_selector->clear();
    for (auto it : _map_fleet_to_robots)
    {
      _fleet_selector->addItem(QString(it.first.c_str()));
    }
  }
}

void RmfPanel::update_end_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _end_waypoint_selector->clear();
  for (const auto& waypoint : graph_info.graph.keys())
  {
    // if (!_workcells_only_checkbox->isChecked() ||
    //   waypoint_has_workcell(waypoint.first, graph_info))
      _end_waypoint_selector->addItem(QString(waypoint.first.c_str()));
  }
}

//==================================================================================
// ROS2 Callbacks
//==================================================================================

void RmfPanel::_fleet_state_callback(const FleetState::SharedPtr msg)
{
  // RCLCPP_INFO(_node->get_logger(), "Received FleetState!");
  bool should_update = false;
  auto fleet_name = msg->name;
  if (_map_fleet_to_robots.find(fleet_name) == _map_fleet_to_robots.end())
  {
    // Fleet is new, load parameters from parameter service
    auto graph_info = load_fleet_graph_info(fleet_name);
    if (graph_info)
    {
      // Update Fleet Graph
      _map_fleet_to_graph_info.insert(
        std::pair<std::string, GraphInfo>(fleet_name, graph_info.value()));
      should_update = true;
    }
  }

  // Update robot states locally
  for (auto robot_state : msg->robots)
  {
    _map_robot_to_state[robot_state.name] = robot_state;
    // TODO(BH): Figure out why make_shared doesn't work?
    // auto robots =
    // std::make_shared<std::vector<std::string>>(_map_fleet_to_robots[fleet_name]);
    auto robots = &_map_fleet_to_robots[fleet_name];
    if (std::find(robots->begin(), robots->end(),
      robot_state.name) == robots->end())
    {
      robots->emplace_back(robot_state.name);
      should_update = true;
    }
  }

  if (should_update)
  {
    Q_EMIT configChanged();
  }
}


} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_visualization_rviz2_plugins::RmfPanel, rviz_common::Panel)
