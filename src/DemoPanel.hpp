#ifndef RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DEMOPANEL_HPP
#define RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DEMOPANEL_HPP

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_default_plugins/tools/point/point_tool.hpp>

#include <std_msgs/msg/bool.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QListView>
#include <QPushButton>
#include <QTextEdit>
#include <QTimeEdit>
#include <QTimer>
#include <QLabel>
#include <QFont>

#include <memory>
#include <mutex>
#include <thread>

namespace rmf_visualization_rviz2_plugins {

class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:

  using Bool = std_msgs::msg::Bool;
  using DestinationRequest = rmf_fleet_msgs::msg::DestinationRequest;
  
  DemoPanel(QWidget* parent = 0);
  ~DemoPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:
  void send_destination_request();
  void publish_emergency_signal();
  void update_bed_selector();
  void update_emergency_state();
  void update_endpoint_selector();


private:
  void create_layout();
  void initialize_publishers();
  // void initialize_subscribers();
  void initialize_qt_connections();

  std::string _requester_id;


  QGroupBox* create_request_group_box();
  QGroupBox* create_emergency_group_box();

  // Options - For configuring certain behaviors in the GUI
  // Labels for the various fields
  QLabel* _bed_name_label;
  QLabel* _end_point_name_label;
  QLabel* _emergency_name_label;
  QLabel* _signal_value_label;

  // Selectors - For targeting agents to accomplish goals
  QComboBox* _bed_selector;
  QComboBox* _end_point_selector;

  // Actions Buttons
  QPushButton* _send_destination_request_button;
  QPushButton* _change_emergency_state_button;

  // QTimer to update fields
  QTimer* _heartbeat;

  bool _emergency_signal = false;

  // Hardcoded Values for DP3 A&E
  std::string known_bed[2] = {"Bed001", "Bed002"};
  std::string defined_endpoints[3] = {"bok_ot", "bok_ward", "triage_room"}; 

  std::thread _thread;
  std::mutex _mutex;
  rclcpp::Node::SharedPtr _node;

  // ROS2 Plumbing
  rclcpp::Publisher<rmf_fleet_msgs::msg::DestinationRequest>::SharedPtr _bed001_destination_pub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::DestinationRequest>::SharedPtr _bed002_destination_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _emergency_state_pub;
 
  // ROS2 callbacks

};
} // namespace rmf_visualization_rviz2_plugins

#endif // RMF_VISUALIZATION_RVIZ2_PLUGINS__SRC__DEMOPANEL_HPP
