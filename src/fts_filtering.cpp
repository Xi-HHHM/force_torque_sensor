#include "force_torque_sensor/fts_filtering.h"

using namespace force_torque_sensor;

ForceTorqueSensorFiltering::ForceTorqueSensorFiltering(ros::NodeHandle& nh, std::string raw_data_topic):
nh_(nh),
calibration_params_{nh.getNamespace()+"/Calibration/Offset"}, 
pub_params_{nh.getNamespace()+"/Publish"}, 
node_params_{nh.getNamespace()+"/Node"}, 
gravity_params_{nh.getNamespace()+"/GravityCompensation/params"}
{   
    ForceTorqueSensorFiltering::prepareNode(raw_data_topic);
    
    while(!is_FirstRawDataDelivered_)
    {
        ros::Duration(0.1).sleep();
    }
    ftPullTimer_.start();
}

void ForceTorqueSensorFiltering::prepareNode(std::string raw_data_topic)
{    
    ROS_INFO_STREAM ("Sensor is using namespace '" << nh_.getNamespace() << "'.");
    // ????
    // reconfigCalibrationSrv_.setCallback(boost::bind(&ForceTorqueSensorFiltering::reconfigureCalibrationRequest, this, _1, _2));

    calibration_params_.fromParamServer();
    pub_params_.fromParamServer();
    node_params_.fromParamServer();
    gravity_params_.fromParamServer();

    transform_frame_ = node_params_.transform_frame;
    sensor_frame_ = node_params_.sensor_frame;
    nodePubFreq = node_params_.ft_pub_freq;
    nodePullFreq = node_params_.ft_pull_freq;

    int calibNMeas;
    calibNMeas=calibration_params_.n_measurements;

    if (calibNMeas <= 0)
    {
        ROS_WARN("Parameter 'Calibration/n_measurements' is %d (<=0) using default: 20", calibNMeas);
        calibrationNMeasurements = 20;
    }
    else
        calibrationNMeasurements = (uint)calibNMeas;

    calibrationTBetween = calibration_params_.T_between_meas;
    m_staticCalibration = calibration_params_.isStatic;

    std::map<std::string,double> forceVal,torqueVal;
    forceVal = calibration_params_.force;
    torqueVal = calibration_params_.torque;

    // mean of the offsets
    m_calibOffset.force.x = forceVal["x"];
    m_calibOffset.force.y = forceVal["y"];
    m_calibOffset.force.z = forceVal["z"];
    m_calibOffset.torque.x = torqueVal["x"];
    m_calibOffset.torque.y = torqueVal["y"];
    m_calibOffset.torque.z = torqueVal["z"];

    srvServer_CalculateAverageMasurement_ = nh_.advertiseService("CalculateAverageMasurement", &ForceTorqueSensorFiltering::srvCallback_CalculateAverageMasurement, this);
    srvServer_CalculateOffset_ = nh_.advertiseService("CalculateOffsets", &ForceTorqueSensorFiltering::srvCallback_CalculateOffset, this);
    // srvServer_DetermineCoordianteSystem_ = nh_.advertiseService("DetermineCoordinateSystem", &ForceTorqueSensorFiltering::srvCallback_DetermineCoordinateSystem, this);
    srvServer_ReCalibrate_ = nh_.advertiseService("Recalibrate", &ForceTorqueSensorFiltering::srvCallback_recalibrate, this);
    srvServer_SetSensorOffset_ = nh_.advertiseService("SetSensorOffset", &ForceTorqueSensorFiltering::srvCallback_setSensorOffset, this);
    RawData_sub_ = nh_.subscribe(raw_data_topic, 1, &ForceTorqueSensorFiltering::RawDataCallBack, this);

    p_tfBuffer = new tf2_ros::Buffer();
    p_tfListener = new tf2_ros::TransformListener(*p_tfBuffer, true);
    init_sensor();

    //Wrench Publisher
    is_pub_gravity_compensated_ = pub_params_.gravity_compensated;
    if(is_pub_gravity_compensated_)
        gravity_compensated_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "gravity_compensated", 1);
    
    is_pub_low_pass_ = pub_params_.low_pass;
    if(is_pub_low_pass_)
        low_pass_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "low_pass", 1);
    
    is_pub_moving_mean_=pub_params_.moving_mean;
    if(is_pub_moving_mean_)
        moving_mean_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "moving_mean", 1);
    
    is_pub_threshold_filtered_ =pub_params_.threshold_filtered;
    if(is_pub_threshold_filtered_)
        threshold_filtered_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "threshold_filtered", 1);
    
    is_pub_transformed_data_  = pub_params_.transformed_data;
    if(is_pub_transformed_data_)
        transformed_data_pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "transformed_data", 1);
    
    // create Timer, 
    ftPullTimer_ = nh_.createTimer(ros::Rate(nodePullFreq), &ForceTorqueSensorFiltering::pullFTData, this, false, false);

    //Median Filter
    if(nh_.hasParam("MovingMeanFilter")) {
        useMovingMean = true;
        moving_mean_filter_->configure(nh_.getNamespace()+"/MovingMeanFilter");
    }

    //Low Pass Filter
    if(nh_.hasParam("LowPassFilter")) {
        useLowPassFilter = true;
        low_pass_filter_->configure(nh_.getNamespace()+"/LowPassFilter");
    }

    //Gravity Compenstation
    if(nh_.hasParam("GravityCompensation")) {
        useGravityCompensator = true;
        gravity_compensator_->configure(nh_.getNamespace()+"/GravityCompensation");
    }

    //Threshold Filter
    if(nh_.hasParam("ThresholdFilter")) {
        useThresholdFilter = true;
        threshold_filter_->configure(nh_.getNamespace()+"/ThresholdFilter");
    }
}

void ForceTorqueSensorFiltering::init_sensor()
{
    // Calibrate sensor
    if (m_staticCalibration)
    {
        ROS_INFO("Using static Calibration Offset from paramter server with parametes Force: x:%f, y:%f, z:%f; Torque: x: %f, y:%f, z:%f;",
        m_calibOffset.force.x, m_calibOffset.force.y, m_calibOffset.force.z,
        m_calibOffset.torque.x, m_calibOffset.torque.y, m_calibOffset.torque.z
        );
        offset_.force.x = m_calibOffset.force.x;
        offset_.force.y = m_calibOffset.force.y;
        offset_.force.z = m_calibOffset.force.z;
        offset_.torque.x= m_calibOffset.torque.x;
        offset_.torque.y = m_calibOffset.torque.y;
        offset_.torque.z = m_calibOffset.torque.z;
    }
    else
    {
        ROS_INFO("Calibrating sensor. Plase wait...");
        geometry_msgs::Wrench temp_offset;
        if (not calibrate(true, &temp_offset))
            std:: string msg = "Calibration failed! :/";
        
    }

apply_offset_ = true;


}

bool ForceTorqueSensorFiltering::srvCallback_CalculateAverageMasurement(force_torque_sensor::CalculateAverageMasurement::Request& req, force_torque_sensor::CalculateAverageMasurement::Response& res)
{
    if (is_FirstRawDataDelivered_)
    {
        res.success = true;
        res.message = "Measurement successfull! :)";
        res.measurement = makeAverageMeasurement(req.N_measurements, req.T_between_meas, req.frame_id);
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised! :/";
    }

    return true;
}

bool ForceTorqueSensorFiltering::srvCallback_CalculateOffset(force_torque_sensor::CalculateSensorOffset::Request& req, force_torque_sensor::CalculateSensorOffset::Response& res)
{
    if (is_FirstRawDataDelivered_)
    {
        if (calibrate(req.apply_after_calculation, &res.offset))
        {
            res.success = true;
            res.message = "Calibration successfull! :)";
        }
        else
        {
            res.success = false;
            res.message = "Calibration failed! :/";
        }
    }
    else
    {
        res.success = false;
        res.message = "FTS not initialised! :/";
    }

    return true;
}

bool ForceTorqueSensorFiltering::srvCallback_recalibrate(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    if (!is_FirstRawDataDelivered_)
    {
        ROS_WARN("FTS-Node is not initialized, please initialize first!");
        res.success = false;
        res.message = "Failed to recalibrate because Node is not initiliazed.";
        return true;
    }
    if (!(nh_.hasParam("force") && nh_.hasParam("CoG_x") && nh_.hasParam("CoG_y") && nh_.hasParam("CoG_z")))
    {
        ROS_ERROR("Cannot use dynamic recalibration without all values for Gravity Compensation, set parameters or use "
                  "'Calibrate' service instead.");
        res.success = false;
        res.message = "Failed to recalibrate because of missing Parameters for Gravity Compensation.";
        return true;
    }
    geometry_msgs::Vector3Stamped gravity, gravity_transformed;
    geometry_msgs::Vector3 cog;
    double force_value;
    cog.x = gravity_params_.CoG_x;
    cog.y = gravity_params_.CoG_y;
    cog.z = gravity_params_.CoG_z;
    force_value = gravity_params_.force;
    gravity.vector.z = -force_value;
    tf2::doTransform(gravity, gravity_transformed, p_tfBuffer->lookupTransform(sensor_frame_, transform_frame_, ros::Time(0)));
    geometry_msgs::Wrench offset;
    calibrate(false, &offset);
    offset_.force.x  -= gravity_transformed.vector.x;
    offset_.force.y  -= gravity_transformed.vector.y;
    offset_.force.z  -= gravity_transformed.vector.z;
    offset_.torque.x -= (gravity_transformed.vector.y * cog.z - gravity_transformed.vector.z * cog.y);
    offset_.torque.y -= (gravity_transformed.vector.z * cog.x - gravity_transformed.vector.x * cog.z);
    offset_.torque.z -= (gravity_transformed.vector.x * cog.y - gravity_transformed.vector.y * cog.x);
    res.success = true;
    res.message = "Successfully recalibrated FTS!";
    return true;
}

bool ForceTorqueSensorFiltering::srvCallback_setSensorOffset(force_torque_sensor::SetSensorOffset::Request &req, force_torque_sensor::SetSensorOffset::Response &res)
{
    offset_.force.x  = req.offset.force.x;
    offset_.force.y  = req.offset.force.y;
    offset_.force.z  = req.offset.force.z;
    offset_.torque.x = req.offset.torque.x;
    offset_.torque.y = req.offset.torque.y;
    offset_.torque.z = req.offset.torque.z;

    res.success = true;
    res.message = "Offset is successfully set!";
    return true;
}


bool ForceTorqueSensorFiltering::calibrate(bool apply_after_calculation, geometry_msgs::Wrench *new_offset)
{
    apply_offset_ = false;
    ROS_INFO("Calibrating using %d measurements and %f s pause between measurements.", calibrationNMeasurements, calibrationTBetween);
    geometry_msgs::Wrench temp_offset = makeAverageMeasurement(calibrationNMeasurements, calibrationTBetween);

    apply_offset_ = true;
    if (apply_after_calculation) {
        offset_ = temp_offset;
    }

    ROS_INFO("Calculated Calibration Offset: Fx: %f; Fy: %f; Fz: %f; Mx: %f; My: %f; Mz: %f", temp_offset.force.x, temp_offset.force.y, temp_offset.force.z, temp_offset.torque.x, temp_offset.torque.y, temp_offset.torque.z);
    *new_offset = temp_offset;

    return true;
}

geometry_msgs::Wrench ForceTorqueSensorFiltering::makeAverageMeasurement(uint number_of_measurements, double time_between_meas, std::string frame_id)
{
    geometry_msgs::Wrench measurement;
    int num_of_errors = 0;
    ros::Duration duration(time_between_meas);
    for (int i = 0; i < number_of_measurements; i++)
    {
      geometry_msgs::Wrench output;
      //std::cout<<"frame id"<< frame_id<<std::endl;
      if (frame_id.compare("") != 0) {
      if (not transform_wrench(frame_id, sensor_frame_, moving_mean_filtered_wrench_.wrench, output))
      {
	  num_of_errors++;
	  if (num_of_errors > 200){
	    return measurement;
	  }
	  i--;
	  continue;
	  }
      }
      else
      {
      	output = moving_mean_filtered_wrench_.wrench;
      }
      measurement.force.x += output.force.x;
      measurement.force.y += output.force.y;
      measurement.force.z += output.force.z;
      measurement.torque.x += output.torque.x;
      measurement.torque.y += output.torque.y;
      measurement.torque.z+= output.torque.z;
      duration.sleep();
    }
    measurement.force.x /= number_of_measurements;
    measurement.force.y /= number_of_measurements;
    measurement.force.z /= number_of_measurements;
    measurement.torque.x /= number_of_measurements;
    measurement.torque.y /= number_of_measurements;
    measurement.torque.z /= number_of_measurements;
    return measurement;
}


void ForceTorqueSensorFiltering::filterFTData(){

    transformed_data_.header.stamp = moving_mean_filtered_wrench_.header.stamp;
    transformed_data_.header.frame_id = transform_frame_;
    if (transform_wrench(transform_frame_, sensor_frame_, moving_mean_filtered_wrench_.wrench, transformed_data_.wrench))
    {
      //gravity compensation
      if(useGravityCompensator)
      {
          gravity_compensator_->update(transformed_data_, gravity_compensated_force_);
      }
      else gravity_compensated_force_ = transformed_data_;

      //treshhold filtering
      if(useThresholdFilter)
      {
          threshold_filter_->update(gravity_compensated_force_, threshold_filtered_force_);
      }
      else threshold_filtered_force_ = gravity_compensated_force_;

      if(is_pub_transformed_data_)
         if (transformed_data_pub_->trylock()){
              transformed_data_pub_->msg_ = transformed_data_;
              transformed_data_pub_->unlockAndPublish();
         }
      if(is_pub_gravity_compensated_ && useGravityCompensator)
         if (gravity_compensated_pub_->trylock()){
              gravity_compensated_pub_->msg_ = gravity_compensated_force_;
              gravity_compensated_pub_->unlockAndPublish();
         }

      if(is_pub_threshold_filtered_ && useThresholdFilter)
         if (threshold_filtered_pub_->trylock()){
             threshold_filtered_pub_->msg_ = threshold_filtered_force_;
             threshold_filtered_pub_->unlockAndPublish();
        }
    }
}

bool ForceTorqueSensorFiltering::transform_wrench(std::string goal_frame, std::string source_frame, geometry_msgs::Wrench wrench, geometry_msgs::Wrench& transformed)
{
  geometry_msgs::TransformStamped transform;

  try
    {
        transform = p_tfBuffer->lookupTransform(goal_frame, source_frame, ros::Time(0));
        _num_transform_errors = 0;
    }
    catch (tf2::TransformException ex)
    {
      if (_num_transform_errors%100 == 0){
            ROS_ERROR("%s", ex.what());
      }
      _num_transform_errors++;
      return false;
    }
	
    tf2::doTransform(wrench, transformed, transform);

    return true;
}

void ForceTorqueSensorFiltering::pullFTData(const ros::TimerEvent &event)
{
    if(is_FirstRawDataDelivered_)
    {
        if (apply_offset_) {
            sensor_data_.wrench.force.x  -= offset_.force.x;
            sensor_data_.wrench.force.y  -= offset_.force.y;
            sensor_data_.wrench.force.z  -= offset_.force.z;
            sensor_data_.wrench.torque.x -= offset_.torque.x;
            sensor_data_.wrench.torque.y -= offset_.torque.y;
            sensor_data_.wrench.torque.z -= offset_.torque.z;
        }

        //lowpass
        low_pass_filtered_data_.header = sensor_data_.header;
        if(useLowPassFilter)
            low_pass_filter_->update(sensor_data_,low_pass_filtered_data_);
        else 
            low_pass_filtered_data_ = sensor_data_;

        //moving_mean
        moving_mean_filtered_wrench_.header = low_pass_filtered_data_.header;
        if(useMovingMean)
            moving_mean_filter_->update(low_pass_filtered_data_, moving_mean_filtered_wrench_);
        else
            moving_mean_filtered_wrench_ = low_pass_filtered_data_;

        if(is_pub_low_pass_)
            if (low_pass_pub_->trylock()){
                low_pass_pub_->msg_ = low_pass_filtered_data_;
                low_pass_pub_->unlockAndPublish();
            }
        
        if(is_pub_moving_mean_)
            if (moving_mean_pub_->trylock()){
                moving_mean_pub_->msg_ = moving_mean_filtered_wrench_;
                moving_mean_pub_->unlockAndPublish();
            }
        
        filterFTData();
    }
}

void ForceTorqueSensorFiltering::RawDataCallBack(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{   
    // ROS_WARN_STREAM ("I am telling you ....");
    sensor_data_ = *msg;
    if(!is_FirstRawDataDelivered_)
        is_FirstRawDataDelivered_ = true; 
}
