#include "force_torque_sensor/fts_filtering.h"
// #include <force_torque_sensor/force_torque_sensor_sim.h>
// #include <force_torque_sensor/NodeConfigurationParameters.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_torque_filtering_node");

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::NodeHandle nh("/fts");

    new force_torque_sensor::ForceTorqueSensorFiltering(nh, "/wrench");

    ROS_INFO("ForceTorque Sensor Filtering Node running.");

    ros::waitForShutdown();

    return 0;
}