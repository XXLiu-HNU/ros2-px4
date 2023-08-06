#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
//#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <functional>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
//#include <pthread.h>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;



float x_position,y_position,z_position;

bool flag = true;
bool dislock = true;
bool finish_hj_task =false; //if finish hangji task
double fly_height = 2.0;//uav fly height 
bool stop_flag = false;
bool rcv_coodrdinate_state =true;
uint64_t HJ_Point;


class Corrdinate
{
public:
	Corrdinate(std::double_t x,std::double_t y)
	{
		this->x = x;
		this->y = y;
	}
	std::double_t x;
	std::double_t y;
};


class OffboardControl : public rclcpp::Node {
public:
		std::vector<std::double_t> my_double_array={20,20,20};//position axis 
		
		OffboardControl() : Node("uav1") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("uav1_control/fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("uav1_control/fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("uav1_control/fmu/vehicle_command/in", 10);

     	
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);
		
     	
#endif
		// get common timestamp
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", qos,
				[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		//初始化信息，接受vehicle_local_position的信息
		//话题名字为"/fmu/out/vehicle_local_position"
		//之后调用回调函数，获取无人机的姿态信息，并进行初始化
		px4_position_subscription = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",qos,
				[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
					x_coordinate_.store(msg->x); 
					y_coordinate_.store(msg->y);
					z_coordinate_.store(msg->z);


					if (arm_able == false)// uav cant takeoff ,get init position
					{
						this->x_init_corrdinate = msg->x;
						this->y_init_corrdinate = msg->y;
						this->z_init_corrdinate = msg->z;
						this->arm_able = true;

						this->vehicle_lat = msg->ref_lat;
						this->vehicle_lon = msg->ref_lon;
						this->vehicle_alt = msg->ref_alt;

						//set_home();
						RCLCPP_INFO(this->get_logger(), "Arm command send");
						cout<<"x_init_corrdinate:" << this->x_init_corrdinate <<endl;
						cout<<"y_init_corrdinate:" << this->y_init_corrdinate <<endl;
						cout<<"z_init_corrdinate:" << this->z_init_corrdinate <<endl;

						cout<<"位置初始化完毕"<<endl;
					}
				});

		desclar_parameter_();
		offboard_setpoint_counter_ = 0;
		rcv_coodrdinate_state=true;
		auto timer_callback = [this]() -> void 
		{
			//std::cout<< "timer_callback in:" <<std::endl;
			offboard_setpoint_counter_++;
			if (offboard_setpoint_counter_ == 10 && arm_able == true)
			{
				this->arm();
				//offboard_setpoint_counter_=0;
			}
			
			get_parameter_(); 

			//control uav to fly by using m_vecMain value
			if (true)//初始位置获取完毕!stop_flag
			{	
			   if(takeoff_land ==0.0 )//起飞
			    {
					
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					publish_offboard_control_mode();
					publish_trajectory_setpoint(my_double_array[0],my_double_array[1], my_double_array[2]);//initative position, very important
					this->arm();
					//cout<<"takeoff_state:"<<this->takeoff_state_<<endl;

				}
			    
			    else if(takeoff_land ==1.0)//降落
			    {
			    	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 11, 0);
					//publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_PRECLAND,0.0,1.0);
					
					//this->disarm();
			    
			    }
			    
			    else if(takeoff_land == 2.0)//返航
			    {
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
					//this->disarm();
			    
			    }


			}


		};
		timer_ = this->create_wall_timer(200ms, timer_callback);


	}
	~OffboardControl()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 11, 0);
	}

	void arm() const;
	void disarm() const;

private:

	std::atomic<uint64_t> timestamp_;   //!< common synced timestampe
	std::atomic<float_t> x_coordinate_;   //!< common synced timestamped
	std::atomic<float_t> y_coordinate_;   //!< common synced timestamped
	std::atomic<float_t> z_coordinate_;

	vector<Corrdinate> m_vecMain;
	bool arm_able = false;              //lock flag    
	bool takeoff_flag = true;           

	float x_init_corrdinate,y_init_corrdinate,z_init_corrdinate; // takeoff  begin position corrdinate       
	uint8_t takeoff_state_;										 // takeoff  state 


	//std::vector<std::double_t> my_double_array;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    uint64_t j;
	float_t takeoff_land = 2;	//起飞标志位，0：起飞，1：降落，2：返航
	float_t x = 0;
	float_t y = 0;
	float_t z = 0;

	int32_t vehicle_lat;
	int32_t vehicle_lon;
	int32_t vehicle_alt;


	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_position_subscription;

	std::vector<std::string> param_names = {"x", "y", "z"};
	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint(float_t x, float_t y ,float_t z) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0,float param7 = 0.0) const;

	void publish_vehicle_command_test(uint32_t command, float_t param1,float_t param2,float_t param3,
													float_t param4,float_t param5,float_t param6,float_t param7) const;

	void desclar_parameter_();
	void get_parameter_(); 
    void set_home();
};


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	//RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::desclar_parameter_()
{
	declare_parameter<std::float_t>("takeoff_land", takeoff_land);
	declare_parameter<std::float_t>("fly_height", fly_height);
	declare_parameter("my_double_array", std::vector<std::double_t>{0.0,0.0});//state uav initative position x:0.0,y:0.0
}

void OffboardControl::get_parameter_() 
{
    get_parameter("takeoff_land", takeoff_land);//fly mode flag: 0.0:takeoff  1.0:land  2.0:return  
    get_parameter("fly_height", fly_height);//get ros2 sent fly_height parameter 
    get_parameter("my_double_array",my_double_array);//get ros2 sent fly position axis. x:  y:
}



/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x,float y,float z) const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	//msg.position={x + x_init_corrdinate,y + y_init_corrdinate,z + z_init_corrdinate};
	msg.yaw = -3.14; // [-PI:PI]
	msg.position = {50.0, 50.0, -50.0};

	trajectory_setpoint_publisher_->publish(msg);
}



/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2,float param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command_test(uint32_t command, 
													float_t param1,float_t param2,float_t param3,
													float_t param4,float_t param5,float_t param6,float_t param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;

	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::set_home()
{

	publish_vehicle_command_test(VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
								1.0,
								0.0,
								0.0,
								NAN,
								this->vehicle_lat,
								this->vehicle_lon,
								this->vehicle_alt
								);

}





int main(int argc, char* argv[]) 
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

