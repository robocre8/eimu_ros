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
    this->declare_parameter<std::string>("frame_id", "imu");
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<double>("publish_frequency", 50.0);
    this->declare_parameter<int>("eimu_reference_frame_id", 1);
    this->declare_parameter<bool>("publish_tf_on_map_frame", false);

    frame_id = this->get_parameter("frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());

    port = this->get_parameter("port").as_string();
    RCLCPP_INFO(this->get_logger(), "port: %s", port.c_str());

    publish_frequency = this->get_parameter("publish_frequency").as_double();
    RCLCPP_INFO(this->get_logger(), "publish_frequency: %f", publish_frequency);

    eimu_reference_frame_id = this->get_parameter("eimu_reference_frame_id").as_int();
    RCLCPP_INFO(this->get_logger(), "eimu_reference_frame_id: %d", eimu_reference_frame_id);

    publish_tf_on_map_frame = this->get_parameter("publish_tf_on_map_frame").as_bool();
    RCLCPP_INFO(this->get_logger(), "publish_tf_on_map_frame: %d", publish_tf_on_map_frame);
    /*---------------------------------------------------------------------*/

    /*----------start connection to eimu_driver module---------------*/
    eimu.connect(port);
    // wait for the imu to fully setup
    for (int i = 1; i <= 4; i += 1)
    {
      delay_ms(1000);
      RCLCPP_INFO(this->get_logger(), "%d", i);
    }

    filterGain = eimu.getFilterGain();
    eimu.setWorldFrameId(eimu_reference_frame_id);
    ref_frame_id = eimu.getWorldFrameId();
    /*---------------------------------------------------------------------*/

    /*----------initialize IMU message---------------*/
    messageImu.header.frame_id = frame_id;

    eimu.readRPYVariance(data_x, data_y, data_z);
    messageImu.orientation_covariance = {data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z};

    eimu.readGyroVariance(data_x, data_y, data_z);
    messageImu.angular_velocity_covariance = {data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z};

    eimu.readAccVariance(data_x, data_y, data_z);
    messageImu.linear_acceleration_covariance = {data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z};
    /*---------------------------------------------------------------------*/

    /*---------------start imu and mag publishers and timer-----------------------------*/
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    imu_rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/data_rpy", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    eimu.clearDataBuffer();
    timer_ = this->create_wall_timer(
        std::chrono::microseconds((long)(1000000 / publish_frequency)),
        std::bind(&EIMU_ROS::publish_imu_callback, this));
    /*---------------------------------------------------------------------*/

    RCLCPP_INFO(this->get_logger(), "eimu_ros node has started with filterGain: %f", filterGain);
    RCLCPP_INFO(this->get_logger(), "eimu_ros node has started with Reference Frame: %s", ref_frame_list[ref_frame_id].c_str());
    if (publish_tf_on_map_frame)
    {
      RCLCPP_INFO(this->get_logger(), "imu transform is being published on map-frame for test rviz viewing");
    }
  }

private:
  void publish_imu_callback()
  {
    messageImu.header.stamp = rclcpp::Clock().now();

    bool success = eimu.readImuData(r, p, y, ax, ay, az, gz, gy, gz);
    if (success) {

    }
    
    rpy.vector.x = r;
    rpy.vector.y = p;
    rpy.vector.z = y;

    // Create TF2 quaternion
    tf2::Quaternion q;
    q.setRPY(r, p, y); 

    // Fill ROS2 message
    geometry_msgs::msg::Quaternion q_msg;
    messageImu.orientation.w = q.w();
    messageImu.orientation.x = q.x();
    messageImu.orientation.y = q.y();
    messageImu.orientation.z = q.z();

    eimu.readAccGyro(ax, ay, az, gx, gy, gz);
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

  std::string frame_id;
  std::string port;
  double publish_frequency;
  int eimu_reference_frame_id;
  std::vector<std::string> ref_frame_list = {"NWU", "ENU", "NED"}; // (0 - NWU,  1 - ENU,  2 - NED)
  bool publish_tf_on_map_frame;

  EIMU eimu;
  float data_x, data_y, data_z;
  float r, p, y, ax, ay, az, gx, gy, gz;
  float filterGain;
  int ref_frame_id;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EIMU_ROS>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}