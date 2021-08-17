#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include "DemoPanel.hpp"
#include "CustomNames.hpp"

namespace rmf_visualization_rviz2_plugins {

//==============================================================================

DemoPanel::DemoPanel(QWidget* parent)
: rviz_common::Panel(parent),
  _requester_id(DemoPanelRequesterId)
{
  _node = std::make_shared<rclcpp::Node>(_requester_id + "_node");
  _heartbeat = new QTimer(this);
  _heartbeat->start(1000);

  create_layout();
  initialize_publishers();
  initialize_qt_connections();
  // For DP3 A&E - Hardcoded way
  update_bed_selector();
  update_endpoint_selector();

  _thread = std::thread([&]()
      {
        rclcpp::spin(_node);
      });
}

//=============================================================================

DemoPanel::~DemoPanel()
{
  if (_thread.joinable())
  {
    _thread.join();
    rclcpp::shutdown();
  }
}

//==============================================================================

void DemoPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

//==============================================================================

void DemoPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

//=============================================================================
// Setting Panel Layout
//=============================================================================

void DemoPanel::create_layout()
{
  QGroupBox* dest_request_gb = create_request_group_box();
  QGroupBox* emergency_state_gb = create_emergency_group_box();
  
  QGridLayout* layout = new QGridLayout;
  layout->addWidget(dest_request_gb, 0, 0, 3, 3);
  layout->addWidget(emergency_state_gb, 3, 0, 2, 3);
  setLayout(layout);
}

//=============================================================================

QGroupBox* DemoPanel::create_request_group_box()
{
  QGroupBox* groupbox = new QGroupBox("Bed Destination Selection");
  QGridLayout* request_layout = new QGridLayout();

  _bed_name_label = new QLabel("Bed ID: ");
  _bed_selector_01 = new QComboBox;
  _bed_selector_02 = new QComboBox;

  _end_point_name_label = new QLabel("End Point: ");
  _end_point_selector_01 = new QComboBox;
  _end_point_selector_01->setEditable(true);
  _end_point_selector_02 = new QComboBox;
  _end_point_selector_02->setEditable(true);

  _send_destination_request_button_01 = new QPushButton("Send Destination Request 01");
  _send_destination_request_button_02 = new QPushButton("Send Destination Request 02");
  request_layout->addWidget(_bed_name_label, 0, 0, 1, 1);
  request_layout->addWidget(_bed_selector_01, 0, 1, 1, 2);
  request_layout->addWidget(_bed_selector_02, 0, 3, 1, 2);
  request_layout->addWidget(_end_point_name_label, 1, 0, 1, 1);
  request_layout->addWidget(_end_point_selector_01, 1, 1, 1, 2);
  request_layout->addWidget(_end_point_selector_02, 1, 3, 1, 2);
  request_layout->addWidget(_send_destination_request_button_01, 2, 1, 1, 2);
  request_layout->addWidget(_send_destination_request_button_02, 2, 3, 1, 2);

  groupbox->setLayout(request_layout);
  return groupbox;
}

//=============================================================================

QGroupBox* DemoPanel::create_emergency_group_box()
{
  QGroupBox* groupbox = new QGroupBox("Emergency State Selection");
  QGridLayout* emergency_layout = new QGridLayout();

  _emergency_name_label = new QLabel("Emergency Signal: ");
  _signal_value_label = new QLabel("false");

  _change_emergency_state_button = new QPushButton("Change Emergency State");

  emergency_layout->addWidget(_emergency_name_label, 0, 0, 1, 1);
  emergency_layout->addWidget(_signal_value_label, 0, 1, 1, 2);
  emergency_layout->addWidget(_change_emergency_state_button, 1, 0, 1, -1);

  groupbox->setLayout(emergency_layout);
  return groupbox;
}

//=============================================================================
// Initialisation Functions
//=============================================================================

// void RmfPanel::initialize_subscribers()
// {
//   _fleet_state_sub = _node->create_subscription<FleetState>(
//     rmf_visualization_rviz2_plugins::FleetStateTopicName, 10,
//     std::bind(&RmfPanel::_fleet_state_callback, this, std::placeholders::_1));

// }

//=============================================================================

void DemoPanel::initialize_publishers()
{

  _emergency_state_pub = _node->create_publisher<Bool>(
    rmf_visualization_rviz2_plugins::EmergencyTopicName, rclcpp::QoS(10));
  
  _bed001_destination_pub = _node->create_publisher<DestinationRequest>(
    rmf_visualization_rviz2_plugins::Bed001DestReqTopicName, rclcpp::QoS(10));

  _bed002_destination_pub = _node->create_publisher<DestinationRequest>(
    rmf_visualization_rviz2_plugins::Bed002DestReqTopicName, rclcpp::QoS(10));
}

//=============================================================================

void DemoPanel::initialize_qt_connections()
{
  connect(this, SIGNAL(configChanged()), this, 
    SLOT(publish_emergency_signal()));
  
  // connect(this, SIGNAL(configChanged()), this, 
  //   SLOT(update_bed_selector()));

  // connect(this, SIGNAL(configChanged()), this, 
  //   SLOT(update_endpoint_selector()));

  connect(_send_destination_request_button_01, SIGNAL(clicked()), this,
    SLOT(send_destination_request_01()));

  connect(_send_destination_request_button_02, SIGNAL(clicked()), this,
    SLOT(send_destination_request_02()));

   connect(_change_emergency_state_button, SIGNAL(clicked()), this,
    SLOT(update_emergency_state()));

  connect(_heartbeat, SIGNAL(timeout()), this,
    SLOT(publish_emergency_signal()));
}

//=============================================================================
// Action Functions
//=============================================================================

void DemoPanel::publish_emergency_signal()
{
  Bool msg;
  msg.data = _emergency_signal;
  _emergency_state_pub->publish(msg);
}

//=============================================================================

void DemoPanel::send_destination_request_01()
{
  std::string bed_id = _bed_selector_01->currentText().toStdString();
  std::string endpoint = _end_point_selector_01->currentText().toStdString();
  std::cout << bed_id << std::endl;
  DestinationRequest request;
  request.robot_name = bed_id;
  request.task_id = endpoint;

  if (bed_id == "Bed001")
  {
    request.fleet_name = "BedFleetA";
    for (int i = 0; i < 3; i++)
    {
      _bed001_destination_pub->publish(request);
    }
    RCLCPP_INFO(_node->get_logger(), "Bed001 Destination Request has been Published");
  }

  if (bed_id == "Bed002")
  {
    request.fleet_name = "BedFleetB";
    for (int i = 0; i < 3; i++)
    {
      _bed002_destination_pub->publish(request);
    }
    RCLCPP_INFO(_node->get_logger(), "Bed002 Destination Request has been Published");
  }
}

void DemoPanel::send_destination_request_02()
{
  std::string bed_id = _bed_selector_02->currentText().toStdString();
  std::string endpoint = _end_point_selector_02->currentText().toStdString();
  std::cout << bed_id << std::endl;
  DestinationRequest request;
  request.robot_name = bed_id;
  request.task_id = endpoint;

  if (bed_id == "Bed001")
  {
    request.fleet_name = "BedFleetA";
    for (int i = 0; i < 3; i++)
    {
      _bed001_destination_pub->publish(request);
    }
    RCLCPP_INFO(_node->get_logger(), "Bed001 Destination Request has been Published");
  }

  if (bed_id == "Bed002")
  {
    request.fleet_name = "BedFleetB";
    for (int i = 0; i < 3; i++)
    {
      _bed002_destination_pub->publish(request);
    }
    RCLCPP_INFO(_node->get_logger(), "Bed002 Destination Request has been Published");
  }
}

//=============================================================================
// Update Functions
//=============================================================================

void DemoPanel::update_emergency_state()
{
  if (_emergency_signal == true)
  {
    _emergency_signal = false;
    _signal_value_label->setText("false");
    RCLCPP_INFO(_node->get_logger(), "Emergency Signal Deactivated!!");
  }
  else
  {
    _emergency_signal = true;
    _signal_value_label->setText("true");
    RCLCPP_INFO(_node->get_logger(), "Emergency Signal Activated!!");
  }
  Q_EMIT configChanged();
}

//=============================================================================
// TO DO: adding graph parsing and fleet reading and sieving
void DemoPanel::update_bed_selector()
{
  for (int i = 0; i < 2; i++)
  {
    _bed_selector_01->addItem(QString((known_bed[i]).c_str()));
    _bed_selector_02->addItem(QString((known_bed[i]).c_str()));
  }
}

void DemoPanel::update_endpoint_selector()
{
  for (int i = 0; i < 4; i++)
  {
    _end_point_selector_01->addItem(QString((defined_endpoints[i]).c_str()));
    _end_point_selector_02->addItem(QString((defined_endpoints[i]).c_str()));
  }
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_visualization_rviz2_plugins::DemoPanel, rviz_common::Panel)