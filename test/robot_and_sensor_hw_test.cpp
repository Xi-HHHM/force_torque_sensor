#include <ros/ros.h>
#include <gtest/gtest.h>
#include <controller_manager/controller_manager.h>
#include <robot_and_sensor_hw/robot_and_sensor_hw.h>

TEST(ForceTorqueSensor, initOk)
{
  ros::NodeHandle nh;

  robot_and_sensor_hw::RobotAndSensorHW hw;
  bool init_success = hw.init(nh, nh);
  ASSERT_TRUE(init_success);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "RobotAndSensorHWTestNode");
  ROS_INFO("Testing robot and sensor hw!");

  int ret = RUN_ALL_TESTS();

  ros::shutdown();
  return ret;
}

