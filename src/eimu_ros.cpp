#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "eimu_ros/eimu.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

class EIMU_ROS : public rclcpp::Node
{
public:
  EIMU_ROS() : Node("eimu_ros")
  {
    /*---------------node parameter declaration-----------------------------*/
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("serial_baud_rate", 115200);
    this->declare_parameter<int>("serial_timeout_ms", 16);
    this->declare_parameter<std::string>("frame_id", "imu");
    this->declare_parameter<double>("publish_frequency", 50.0);
    this->declare_parameter<int>("world_frame_id", 1);
    this->declare_parameter<bool>("publish_tf_on_map_frame", false);
    this->declare_parameter<bool>("use_static_covariances", false);
    std::vector<double> default_covariance_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->declare_parameter("static_covariance_orientation", default_covariance_values);
    this->declare_parameter("static_covariance_angular_velocity", default_covariance_values);
    this->declare_parameter("static_covariance_linear_acceleration", default_covariance_values);

    serial_port = this->get_parameter("serial_port").as_string();
    RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());

    serial_baud_rate = this->get_parameter("serial_baud_rate").as_int();
    RCLCPP_INFO(this->get_logger(), "serial_baud_rate: %d", serial_baud_rate);

    serial_timeout_ms = this->get_parameter("serial_timeout_ms").as_int();
    RCLCPP_INFO(this->get_logger(), "serial_timeout_ms: %d", serial_timeout_ms);

    frame_id = this->get_parameter("frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());

    publish_frequency = this->get_parameter("publish_frequency").as_double();
    RCLCPP_INFO(this->get_logger(), "publish_frequency: %f", publish_frequency);

    world_frame_id = this->get_parameter("world_frame_id").as_int();
    RCLCPP_INFO(this->get_logger(), "world_frame_id: %d", world_frame_id);

    publish_tf_on_map_frame = this->get_parameter("publish_tf_on_map_frame").as_bool();
    RCLCPP_INFO(this->get_logger(), "publish_tf_on_map_frame: %d", publish_tf_on_map_frame);

    use_static_covariances = this->get_parameter("use_static_covariances").as_bool();
    RCLCPP_INFO(this->get_logger(), "use_static_covariances: %d", use_static_covariances);

    static_covariance_orientation = this->get_parameter("static_covariance_orientation").as_double_array();
    static_covariance_angular_velocity = this->get_parameter("static_covariance_angular_velocity").as_double_array();
    static_covariance_linear_acceleration = this->get_parameter("static_covariance_linear_acceleration").as_double_array();
    
    /*---------------------------------------------------------------------*/

    /*----------start connection to eimu_driver module---------------*/
    eimu.connect(serial_port, serial_baud_rate, serial_timeout_ms);

    // wait for the imu to fully setup
    for (int i = 1; i <= 4; i += 1)
    {
      delay_ms(1000);
      RCLCPP_INFO(this->get_logger(), "%d", i);
    }

    // success = eimu.clearDataBuffer();

    std::tie(success, val0) = eimu.getFilterGain();
    if (success) 
      filterGain = val0;

    eimu.setWorldFrameId(world_frame_id);

    std::tie(success, val0) = eimu.getWorldFrameId();
    if (success)
      world_frame_id = val0;
    /*---------------------------------------------------------------------*/

    /*----------initialize IMU message---------------*/
    messageImu.header.frame_id = frame_id;

    if (use_static_covariances){
      messageImu.orientation_covariance = {static_covariance_orientation.at(0), 0.0, 0.0, 
                                           0.0, static_covariance_orientation.at(4), 0.0, 
                                           0.0, 0.0, static_covariance_orientation.at(8)};

      messageImu.angular_velocity_covariance = {static_covariance_angular_velocity.at(0), 0.0, 0.0, 
                                                0.0, static_covariance_angular_velocity.at(4), 0.0, 
                                                0.0, 0.0, static_covariance_angular_velocity.at(8)};

      messageImu.linear_acceleration_covariance = {static_covariance_linear_acceleration.at(0), 0.0, 0.0, 
                                                   0.0, static_covariance_linear_acceleration.at(4), 0.0, 
                                                   0.0, 0.0, static_covariance_linear_acceleration.at(8)};

    } else {
      std::tie(success, val0, val1, val2) = eimu.readRPYVariance();
      if(success){
        data_x = val0;
        data_y = val1;
        data_z = val2;
      } else {
        RCLCPP_WARN(this->get_logger(), "WARNING: Could not read IMU orientation variance, defaulting to Static orientation covariance");
        data_x = static_covariance_orientation.at(0);
        data_y = static_covariance_orientation.at(4);
        data_z = static_covariance_orientation.at(8);
      }
      messageImu.orientation_covariance = {data_x, 0.0, 0.0,
                                           0.0, data_y, 0.0,
                                           0.0, 0.0, data_z};

      std::tie(success, val0, val1, val2) = eimu.readGyroVariance();
      if(success){
        data_x = val0;
        data_y = val1;
        data_z = val2;
      } else {
        RCLCPP_WARN(this->get_logger(), "WARNING: Could not read IMU Gyro variance, defaulting to Static angular vel covariance");
        data_x = static_covariance_angular_velocity.at(0);
        data_y = static_covariance_angular_velocity.at(4);
        data_z = static_covariance_angular_velocity.at(8);
      }
      messageImu.angular_velocity_covariance = {data_x, 0.0, 0.0,
                                                0.0, data_y, 0.0,
                                                0.0, 0.0, data_z};

      std::tie(success, val0, val1, val2) = eimu.readAccVariance();
      if(success){
        data_x = val0;
        data_y = val1;
        data_z = val2;
      } else {
        RCLCPP_WARN(this->get_logger(), "WARNING: Could not read IMU acceleration variance, defaulting to Static acceleration covariance");
        data_x = static_covariance_linear_acceleration.at(0);
        data_y = static_covariance_linear_acceleration.at(4);
        data_z = static_covariance_linear_acceleration.at(8);
      }
      messageImu.linear_acceleration_covariance = {data_x, 0.0, 0.0,
                                                   0.0, data_y, 0.0,
                                                   0.0, 0.0, data_z};
    }

    RCLCPP_INFO(this->get_logger(), "orientation_covariance: [%f, ..., %f, ..., %f]", messageImu.orientation_covariance[0], messageImu.orientation_covariance[4], messageImu.orientation_covariance[8]);
    RCLCPP_INFO(this->get_logger(), "angular_velocity_covariance: [%f, ..., %f, ..., %f]", messageImu.angular_velocity_covariance[0], messageImu.angular_velocity_covariance[4], messageImu.angular_velocity_covariance[8]);
    RCLCPP_INFO(this->get_logger(), "linear_acceleration_covariance: [%f, ..., %f, ..., %f]", messageImu.linear_acceleration_covariance[0], messageImu.linear_acceleration_covariance[4], messageImu.linear_acceleration_covariance[8]);
    /*---------------------------------------------------------------------*/

    /*---------------start imu and mag publishers and timer-----------------------------*/
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    imu_rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/data_rpy", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        std::chrono::microseconds((long)(1000000 / publish_frequency)),
        std::bind(&EIMU_ROS::publish_imu_callback, this));
    /*---------------------------------------------------------------------*/

    RCLCPP_INFO(this->get_logger(), "eimu_ros node has started with filterGain: %f", filterGain);
    RCLCPP_INFO(this->get_logger(), "eimu_ros node has started with Reference Frame: %s", world_frame_list[world_frame_id].c_str());
    if (publish_tf_on_map_frame)
    {
      RCLCPP_INFO(this->get_logger(), "imu transform is being published on map-frame for test rviz viewing");
    }
  }


private:
  void publish_imu_callback()
  {
    messageImu.header.stamp = rclcpp::Clock().now();

    std::tie(success, val0, val1, val2, val3, val4, val5, val6, val7, val8) = eimu.readImuData();
    if (success){
      r = val0; p = val1; y = val2;
      ax = val3; ay = val4; az = val5;
      gx = val6; gy = val7; gz = val8;
    }

    rpy.vector.x = r;
    rpy.vector.y = p;
    rpy.vector.z = y;

    // Create TF2 quaternion
    tf2::Quaternion q;
    q.setRPY(r, p, y); 

    messageImu.orientation.w = q.w();
    messageImu.orientation.x = q.x();
    messageImu.orientation.y = q.y();
    messageImu.orientation.z = q.z();

    // Fill ROS2 message

    // messageImu.orientation.w = qw;
    // messageImu.orientation.x = qx;
    // messageImu.orientation.y = qy;
    // messageImu.orientation.z = qz;

    messageImu.angular_velocity.x = gx;
    messageImu.angular_velocity.y = gy;
    messageImu.angular_velocity.z = gz;

    messageImu.linear_acceleration.x = ax;
    messageImu.linear_acceleration.y = ay;
    messageImu.linear_acceleration.z = az;

    
    // tf2::Matrix3x3(tf2::Quaternion(
    //                    messageImu.orientation.x,
    //                    messageImu.orientation.y,
    //                    messageImu.orientation.z,
    //                    messageImu.orientation.w))
    //     .getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    
    rpy.header = messageImu.header;

    if (publish_tf_on_map_frame)
    {
      publish_imu_tf(messageImu);
    }

    imu_data_publisher_->publish(messageImu);
    imu_rpy_publisher_->publish(rpy);
  }

  void publish_imu_tf(sensor_msgs::msg::Imu messageImu)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = messageImu.header.stamp;

    t.header.frame_id = "map";
    t.child_frame_id = messageImu.header.frame_id;

    t.transform.rotation.w = messageImu.orientation.w;
    t.transform.rotation.x = messageImu.orientation.x;
    t.transform.rotation.y = messageImu.orientation.y;
    t.transform.rotation.z = messageImu.orientation.z;

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_rpy_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu messageImu = sensor_msgs::msg::Imu();
  geometry_msgs::msg::Vector3Stamped rpy;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string serial_port;
  int serial_baud_rate;
  int serial_timeout_ms;
  std::string frame_id;
  double publish_frequency;
  int world_frame_id;
  std::vector<std::string> world_frame_list = {"NWU", "ENU", "NED"}; // (0 - NWU,  1 - ENU,  2 - NED)
  bool publish_tf_on_map_frame;
  bool use_static_covariances;
  std::vector<double> static_covariance_orientation;
  std::vector<double> static_covariance_angular_velocity;
  std::vector<double> static_covariance_linear_acceleration;


  EIMU eimu;
  float data_x, data_y, data_z;
  float qw, qx, qy, qz;
  float r, p, y, ax, ay, az, gx, gy, gz;
  float filterGain;

  bool success;
  float val0, val1, val2, val3, val4, val5, val6, val7, val8;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EIMU_ROS>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}