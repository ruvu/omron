/**
Software License Agreement (BSD)

\file      os32c_node.cpp
\authors   Kareem Shehata <kareem@shehata.ca>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <boost/shared_ptr.hpp>
#include <boost/range/algorithm.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// TODO: figure out what functionality rosconsole_bridge was providing
//#include <rosconsole_bridge/bridge.h>
// REGISTER_ROSCONSOLE_BRIDGE;

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"
#include "omron_os32c_driver/os32c.h"
#include "omron_os32c_driver/range_and_reflectance_measurement.h"

using std::cout;
using std::endl;
using namespace std::chrono_literals;
using boost::shared_ptr;
using boost::range::reverse;
using sensor_msgs::msg::LaserScan;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;
using namespace omron_os32c_driver;
using namespace diagnostic_updater;
const double EPS = 1e-3;


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("os32c");

  node->declare_parameter("host", "192.168.1.1");
  node->declare_parameter("local_ip", "0.0.0.0");
  node->declare_parameter("frame_id", "laser");
  node->declare_parameter("start_angle", OS32C::ANGLE_MAX);
  node->declare_parameter("end_angle", OS32C::ANGLE_MIN);
  node->declare_parameter("frequency", 12.856);
  node->declare_parameter("frequency_tolerance", 0.1);
  node->declare_parameter("timestamp_min_acceptable", -1.0);
  node->declare_parameter("timestamp_max_acceptable", -1.0);
  node->declare_parameter("reconnect_timeout", 2.0);
  node->declare_parameter("publish_intensities", false);
  node->declare_parameter("invert_scan", false);

  string host = node->get_parameter("host").as_string();
  string local_ip = node->get_parameter("local_ip").as_string();
  string frame_id = node->get_parameter("frame_id").as_string();
  double start_angle = node->get_parameter("start_angle").as_double();
  double end_angle = node->get_parameter("end_angle").as_double();
  double frequency = node->get_parameter("frequency").as_double();
  double frequency_tolerance = node->get_parameter("frequency_tolerance").as_double();
  double timestamp_min_acceptable = node->get_parameter("timestamp_min_acceptable").as_double();
  double timestamp_max_acceptable = node->get_parameter("timestamp_max_acceptable").as_double();
  double reconnect_timeout = node->get_parameter("reconnect_timeout").as_double();
  bool publish_intensities = node->get_parameter("publish_intensities").as_bool();
  bool invert_scan = node->get_parameter("invert_scan").as_bool();

  node->declare_parameter("expected_frequency", frequency);
  double expected_frequency = node->get_parameter("expected_frequency").as_double();

  // publisher for laserscans
  auto laserscan_pub = node->create_publisher<LaserScan>("scan", 1);

  // Validate frequency parameters
  if (frequency > 25)
  {
    RCLCPP_FATAL(node->get_logger(), "Frequency exceeds the limit of 25hz.");
    return -1;
  }
  else if (frequency <= 0)
  {
    RCLCPP_FATAL(node->get_logger(), "Frequency should be positive");
    return -1;
  }

  if (fabs(frequency - expected_frequency) > EPS)
  {
    RCLCPP_WARN(node->get_logger(), "Frequency parameter is not equal to expected frequency parameter.");
  }

  rclcpp::Rate loop_rate(frequency);
  rclcpp::Rate reconnect_rate(1 / frequency);


  // diagnostics for frequency
  Updater updater(node);
  updater.setHardwareID(host);
  DiagnosedPublisher<LaserScan> diagnosed_publisher(
      laserscan_pub, updater, FrequencyStatusParam(&expected_frequency, &expected_frequency, frequency_tolerance),
      TimeStampStatusParam(timestamp_min_acceptable, timestamp_max_acceptable));

  while (rclcpp::ok())
  {
    boost::asio::io_service io_service;
    shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
    shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 2222, local_ip));
    OS32C os32c(socket, io_socket);

    try
    {
      RCLCPP_INFO(node->get_logger(), "Trying to connect to %s", host.c_str());
      os32c.open(host);
      RCLCPP_INFO(node->get_logger(), "Socket successfuly opened");
    }
    catch (std::runtime_error ex)
    {
      RCLCPP_ERROR(node->get_logger(), "Exception caught opening session: %s. Reconnecting in %.2f seconds ...",
                   ex.what(), reconnect_timeout);
      reconnect_rate.sleep();
      continue;
    }

    try
    {
      os32c.setRangeFormat(RANGE_MEASURE_50M);
      os32c.setReflectivityFormat(REFLECTIVITY_MEASURE_TOT_4PS);
      os32c.selectBeams(start_angle, end_angle);
    }
    catch (std::invalid_argument ex)
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Invalid arguments in sensor configuration: %s. Reconnecting in %.2f seconds ...", ex.what(),
                   reconnect_timeout);
      reconnect_rate.sleep();
      continue;
    }

    LaserScan laserscan_msg;
    os32c.fillLaserScanStaticConfig(&laserscan_msg);
    laserscan_msg.header.frame_id = frame_id;

    while (rclcpp::ok())
    {
      try
      {
        // Poll ranges and reflectivity
        RangeAndReflectanceMeasurement report = os32c.getSingleRRScan();

        // Invert range measurements if z-axis is needed to point upwards.
        if (invert_scan)
        {
          reverse(report.range_data);
        }

        OS32C::convertToLaserScan(report, &laserscan_msg);

        // In earlier versions reflectivity was not received. So to be backwards
        // compatible clear reflectivity from msg.
        if (!publish_intensities)
        {
          laserscan_msg.intensities.clear();
        }

        // Stamp and publish message diagnosed
        laserscan_msg.header.stamp = node->now();
        diagnosed_publisher.publish(laserscan_msg);

        // Update diagnostics
        updater.force_update();
      }
      catch (std::runtime_error ex)
      {
        RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 5.0,
                                     "Exception caught requesting scan data: " << ex.what());
      }
      catch (std::logic_error ex)
      {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Problem parsing return data: " << ex.what());
      }

      if (!laserscan_msg.header.stamp.nanosec == 0 &&
          (node->get_clock()->now() - laserscan_msg.header.stamp).seconds() > reconnect_timeout)
      {
        RCLCPP_ERROR(node->get_logger(), "No scan received for %.2f seconds, reconnecting ...", reconnect_timeout);
        laserscan_msg.header.stamp = rclcpp::Time(0);
        break;
      }

      rclcpp::spin_some(node);

      // sleep
      loop_rate.sleep();
    }

    if (!rclcpp::ok())
    {
      os32c.closeActiveConnection();
      os32c.close();
    }
  }

  rclcpp::shutdown();
}
