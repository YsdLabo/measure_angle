#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TrapezoidalRule  :  public  rclcpp::Node
{
public:
	TrapezoidalRule( ) : Node("trapezoidal_rule_node")
	{
		// サブスクライバーを作成（/imuトピックを購読）
		sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu> (
			"/imu", 
			rclcpp::SensorDataQoS(),
			std::bind(&TrapezoidalRule::callback_imu, this, std::placeholders::_1)
		);
		// パブリッシャーを作成（/trape_angle_zトピックを配信）
		pub_angle_ = this->create_publisher<geometry_msgs::msg::PoseStamped> (
			"/trape_angle_z",
			rclcpp::SensorDataQoS()
		);
	}

private:
	// 購読用コールバック関数
	void  callback_imu( const  sensor_msgs::msg::Imu::SharedPtr  msg )
	{
		sensor_msgs::msg::Imu imu_raw = *msg;	// 受信メッセージをコピー

		rclcpp::Time  current_stamp(imu_raw.header.stamp);

		// 区分求積法
		if(!first_time_) {
			double  h = (current_stamp - last_stamp_).seconds();
			angle_z_ += // ここに台形近似の式を入力。last_raw_z_を使用すること
		}
		else {
			angle_z_ = 0.0;
			first_time_ = false;
		}
		last_raw_z_ = imu_raw.angular_velocity.z;    // 現在の計測値を記憶
		RCLCPP_INFO(this->get_logger(), "angle_z : %f", angle_z_);

		// Yaw角の配信
		tf2::Quaternion  quat;
		quat.setRPY( 0.0, 0.0, angle_z_ );
		quat.normalize();
		geometry_msgs::msg::PoseStamped  pub_msg;
		pub_msg.pose.orientation = tf2::toMsg(quat);
		pub_msg.header.stamp = current_stamp;
		pub_msg.header.frame_id = "imu_link";
		pub_angle_->publish(pub_msg);		

		last_stamp_ = current_stamp;
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_angle_;
	double  angle_z_ = 0.0;
	double  last_raw_z_;
	bool  first_time_ = true;
	rclcpp::Time  last_stamp_;
};

int  main(int  argc, char**  argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrapezoidalRule>());
	rclcpp::shutdown();
	return  0;
}

