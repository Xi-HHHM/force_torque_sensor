#ifndef FTS_FILTERING_INCLUDEDEF_H
#define FTS_FTLTERING_INCLUDEDEF_H

#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Trigger.h>
#include <force_torque_sensor/CalculateAverageMasurement.h>
#include <force_torque_sensor/CalculateSensorOffset.h>
#include <force_torque_sensor/DiagnosticVoltages.h>
#include <force_torque_sensor/SetSensorOffset.h>


#include <iirob_filters/gravity_compensation.h>
#include <iirob_filters/GravityCompensationParameters.h>
#include <iirob_filters/low_pass_filter.h>
#include <iirob_filters/threshold_filter.h>
#include <iirob_filters/moving_mean_filter.h>

#include <math.h>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <force_torque_sensor/HWCommunicationConfigurationParameters.h>
#include <force_torque_sensor/PublishConfigurationParameters.h>
#include <force_torque_sensor/NodeConfigurationParameters.h>
#include <force_torque_sensor/CalibrationParameters.h>
#include <force_torque_sensor/CalibrationConfig.h>

#include <filters/filter_chain.h>
#include <filters/filter_base.h>
#include <filters/mean.h>
#include <realtime_tools/realtime_publisher.h>
// #include "force_torque_sensor/force_torque_sensor_handle.h"

#define PI 3.14159265

namespace force_torque_sensor
{
    class ForceTorqueSensorFiltering
    {
    private:
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *gravity_compensated_pub_;
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *threshold_filtered_pub_;
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *transformed_data_pub_;
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *low_pass_pub_;
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *moving_mean_pub_;

        force_torque_sensor::PublishConfigurationParameters pub_params_;
        force_torque_sensor::NodeConfigurationParameters node_params_;
        force_torque_sensor::CalibrationParameters calibration_params_;
        iirob_filters::GravityCompensationParameters gravity_params_;

        ros::ServiceServer srvServer_CalculateAverageMasurement_;
        ros::ServiceServer srvServer_CalculateOffset_;
        ros::ServiceServer srvServer_DetermineCoordianteSystem_;
        ros::ServiceServer srvServer_ReCalibrate_;
        ros::ServiceServer srvServer_SetSensorOffset_;

        ros::Subscriber RawData_sub_;
        tf2_ros::Buffer *p_tfBuffer;
        tf2_ros::TransformListener *p_tfListener;
        ros::Timer ftPullTimer_;


        geometry_msgs::WrenchStamped gravity_compensated_force_;
        geometry_msgs::WrenchStamped moving_mean_filtered_wrench_;
        geometry_msgs::WrenchStamped threshold_filtered_force_;
        geometry_msgs::WrenchStamped transformed_data_;
        geometry_msgs::WrenchStamped sensor_data_;
        geometry_msgs::WrenchStamped low_pass_filtered_data_;
        geometry_msgs::Wrench offset_;

        filters::FilterBase<geometry_msgs::WrenchStamped> *moving_mean_filter_ = new iirob_filters::MovingMeanFilter<geometry_msgs::WrenchStamped>();
        filters::FilterBase<geometry_msgs::WrenchStamped> *low_pass_filter_ = new iirob_filters::LowPassFilter<geometry_msgs::WrenchStamped>();
        filters::FilterBase<geometry_msgs::WrenchStamped> *threshold_filter_ = new iirob_filters::ThresholdFilter<geometry_msgs::WrenchStamped>();
        filters::FilterBase<geometry_msgs::WrenchStamped> *gravity_compensator_ = new iirob_filters::GravityCompensator<geometry_msgs::WrenchStamped>();

        bool is_FirstRawDataDelivered_;
        bool apply_offset_;

        ros::NodeHandle nh_;
        std::string sensor_frame_, transform_frame_;

        bool is_pub_gravity_compensated_ = false;
        bool is_pub_threshold_filtered_ = false;
        bool is_pub_transformed_data_ = false;
        bool is_pub_sensor_data_ = false;
        bool is_pub_low_pass_ = false;
        bool is_pub_moving_mean_ = false;

        bool useGravityCompensator=false;
        bool useThresholdFilter=false;
        bool useMovingMean = false;
        bool useLowPassFilter = false;

        // HWComm parameters
        int HWCommType; // Only important if can is used
        std::string HWCommPath;
        int HWCommBaudrate;
        int ftsBaseID;
        double nodePubFreq, nodePullFreq;
        uint calibrationNMeasurements;
        double calibrationTBetween;
        int coordinateSystemNMeasurements;
        int coordinateSystemTBetween;
        int coordinateSystemPushDirection;

        uint _num_transform_errors;

  // Variables for Static offset
        bool m_staticCalibration;
        geometry_msgs::Wrench m_calibOffset;
        

        void init_sensor();
        // bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool srvCallback_CalculateOffset(force_torque_sensor::CalculateSensorOffset::Request &req, force_torque_sensor::CalculateSensorOffset::Response &res);
        bool srvCallback_CalculateAverageMasurement(force_torque_sensor::CalculateAverageMasurement::Request &req, force_torque_sensor::CalculateAverageMasurement::Response &res);
        // bool srvCallback_DetermineCoordinateSystem(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        // bool srvReadDiagnosticVoltages(force_torque_sensor::DiagnosticVoltages::Request &req, force_torque_sensor::DiagnosticVoltages::Response &res);
        bool srvCallback_recalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool srvCallback_setSensorOffset(force_torque_sensor::SetSensorOffset::Request &req, force_torque_sensor::SetSensorOffset::Response &res);
        
        geometry_msgs::Wrench makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id="");
        bool calibrate(bool apply_after_calculation,  geometry_msgs::Wrench *new_offset);
        



    public:
        ForceTorqueSensorFiltering(ros::NodeHandle &nh, std::string raw_data_topic);
        void prepareNode(std::string output_frame);
        void pullFTData(const ros::TimerEvent &event);
        void filterFTData();
        void RawDataCallBack(const geometry_msgs::WrenchStamped::ConstPtr& msg);
        void reconfigureCalibrationRequest(force_torque_sensor::CalibrationConfig& config, uint32_t level);
        bool transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench& transformed);

        



    };
}
#endif