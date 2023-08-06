
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
//#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
//#include <px4_msgs/msg/takeoff_status.hpp>
//#include <px4_msgs/msg/position_setpoint.hpp>
//#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>


#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//这里有错误，识别不出来！！！
//time:0803 解决，添加路径、依赖

#include "uav_interfaces/srv/uav_control.hpp"

#include <stdint.h>
#include <functional>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <memory>
#include <string>
#include <thread>


using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;
using std::placeholders::_2;



std::atomic<uint32_t>corrdinate_vect_x;
std::atomic<uint32_t>corrdinate_vect_y;
std::atomic<uint32_t>corrdinate_vect_z;
std::atomic<int>corrdinate_takeoff_land;

std::atomic<int>corrdinate_x_deta;
std::atomic<int>corrdinate_y_deta;
std::atomic<int>corrdinate_z_deta;


float x_position,y_position,z_position;

bool flag = true;
bool dislock = true;
bool finish_hj_task =false; //if finish hangji task
double fly_height = 2.0;//uav fly height 


uint64_t HJ_Point;




class DualThreadedNode : public rclcpp::Node
{
public:
  DualThreadedNode()
  : Node("DualThreadedNode")
  {
    /* These define the callback groups
     * They don't really do much on their own, but they have to exist in order to
     * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
     */
    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);


    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;

	//这里订阅的谁的消息？"uav1_obj_position"
	//time:0804 解决，在“suanfa/calculate”里
    subscription1_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
      "uav1_obj_position",
      rclcpp::QoS(10),
      // std::bind is sort of C++'s way of passing a function
      // If you're used to function-passing, skip these comments
      std::bind(
        &DualThreadedNode::subscriber1_cb,  // First parameter is a reference to the function
        this,                               // What the function should be bound to
        std::placeholders::_1),             // At this point we're not positive of all the
                                            // parameters being passed
                                            // So we just put a generic placeholder
                                            // into the binder
                                            // (since we know we need ONE parameter)
      sub1_opt);                  // This is where we set the callback group.
                                  // This subscription will run with callback group subscriber1

      std::cout<<"subscribe corrdinate begin:"<<endl;

  }

private:
  /**
   * Simple function for generating a timestamp
   * Used for somewhat ineffectually demonstrating that the multithreading doesn't cripple performace
   */
  std::string timing_string()
  {
    rclcpp::Time time = this->now();
    return std::to_string(time.nanoseconds());
  }

  /**
   * Every time the Publisher publishes something, all subscribers to the topic get poked
   * This function gets called when Subscriber1 is poked (due to the std::bind we used when defining it)
   */
  void subscriber1_cb(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
  {
    auto message_received_at = timing_string();

    // Extract current thread
    RCLCPP_INFO(this->get_logger(), "Subscribe corrdinate x:'%d' y:'%d' z:'%d'",msg->data[0],msg->data[1],msg->data[2]);

	corrdinate_vect_x.store(msg->data[0]);
	corrdinate_vect_y.store(msg->data[1]);
	corrdinate_vect_z.store(msg->data[2]);
    cout<<"corrdinate_vect subscription finish"<<endl;
    
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscription1_;

};


class User : public rclcpp :: Node
{
public:
      User(): Node("User")
      {

		position_callback_subscriber1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = position_callback_subscriber1_;
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    position_subscription1_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",qos,
				[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){

	
					uav_position_x.store(msg->x);
			        uav_position_y.store(msg->y);
			        uav_position_z.store(msg->z);	
				});

 	publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("uav_position", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::Float32MultiArray();
		
        message.data = {uav_position_x.load(),uav_position_y.load(),uav_position_z.load()};
        //cout<<"这里是185行，测试输出飞机位置是否正常"<<uav_position_x.load()<<uav_position_y.load()<<uav_position_z.load()<<endl;

        //RCLCPP_INFO(this->get_logger(), "\nCorrdinate Publishing x:'%f' y:'%f' z:'%f'",message.data[0],message.data[1],message.data[2]);
        this->publisher_->publish(message);
      };
    	timer_ = this->create_wall_timer(1000ms, timer_callback);
 

      }


private:
      	rclcpp::TimerBase::SharedPtr timer_;
	    rclcpp::CallbackGroup::SharedPtr position_callback_subscriber1_;
  		rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription1_;
	    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

		std::atomic<float>uav_position_x;
		std::atomic<float>uav_position_y;
		std::atomic<float>uav_position_z;

		

};



class OffboardControl : public rclcpp::Node 
{
public:
		std::vector<std::double_t> my_double_array;//position axis 

		OffboardControl() : Node("uav1_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("uav1_control/fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("uav1_control/fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("uav1_control/fmu/vehicle_command/in", 10);
		
		server_ = this->create_service<uav_interfaces::srv::UavControl>("uav1_control",
                            std::bind(&OffboardControl::uav_control_callback,this,_1,_2));
		client_ = this->create_client<uav_interfaces::srv::UavControl>("uav1_control");

     	
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode",10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);

		server_ = this->create_service<uav_interfaces::srv::UavControl>("uav1_control",
            std::bind(&OffboardControl::uav_control_callback,this,_1,_2));
		client_ = this->create_client<uav_interfaces::srv::UavControl>("uav1_control");
 	
#endif
		// get common timestamp
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync", qos,
				[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		px4_position_subscription = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",qos,
				[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){

					if (arm_able == false)// uav cant takeoff ,get init position
					{
						this->x_init_corrdinate = msg->x;
						this->y_init_corrdinate = msg->y;
						this->z_init_corrdinate = msg->z;
						this->arm_able = true;

						// this->vehicle_lat = msg->ref_lat;
						// this->vehicle_lon = msg->ref_lon;
						// this->vehicle_alt = msg->ref_alt;

						//set_home();
						cout<< this->x_init_corrdinate << this->y_init_corrdinate <<z_init_corrdinate <<endl;
						cout<<"位置初始化完毕"<<endl;
					}
				});

		corrdinate_subscription_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>("uav1_obj_position",10,
		[this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
		{


			corrdinate_vect_x.store(msg->data[0]);
			corrdinate_vect_y.store(msg->data[1]);
			corrdinate_vect_z.store(msg->data[2]);
			//cout<<"corrdinate_vect subscription finish"<<endl;

			//增加request部分
			auto request = std::make_shared<uav_interfaces::srv::UavControl::Request>();
			request->takeoff_land=1;
			request->x_deta=msg->data[0];
			request->y_deta=msg->data[1];
			request->z_deta=msg->data[2];
			auto result = client_->async_send_request(request);

		
			//cout<<"corrdinate_vect_x:"<<corrdinate_vect_x.load()<<"corrdinate_vect_y:"<<corrdinate_vect_y.load()<<"corrdinate_vect_z:"<<corrdinate_vect_z.load()<<endl;
					
		});

		desclar_parameter_();
		offboard_setpoint_counter_ = 0;
		get_parameter_(); 
	
		auto timer_callback = [this]() -> void 
		{

			offboard_setpoint_counter_++;
			if (offboard_setpoint_counter_ == 10 && arm_able == true)
			{
				this->arm();
				//arm_able = false;
				//offboard_setpoint_counter_=0;
			}
			
			//get_parameter_(); 
			vector<double> m_vecSub;
			takeoff_land = corrdinate_takeoff_land.load();
			
			if (takeoff_land == 6)//解锁标志位
			{
			    this->arm();			
			}
			
			if (true)//初始位置获取完毕!stop_flag
			{	
			   if(takeoff_land ==1 && takeoff_flag ==true)//起飞
			    {
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					publish_offboard_control_mode();
					publish_trajectory_setpoint(corrdinate_vect_x.load(),corrdinate_vect_y.load(), corrdinate_vect_z.load());
					//publish_trajectory_setpoint(0,0,4);

				}
			    
			    else if(takeoff_land ==2)//降落
			    {
			    	//publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 11, 0);
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
					//takeoff_flag =false;
					//px4_msgs::msg::VehicleLocalPosition::SharedPtr msg;
	
					//this->disarm();
			    
			    }
			    
			    else if(takeoff_land == 3)//返航
			    {
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
					//this->disarm();
			    
			    }
			    
			    else if(takeoff_land == 4)//增量步进运动
			    {

			               //get_parameter_();
			                  
			                   
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					publish_offboard_control_mode();
					
					rclcpp::Parameter int_param1 = this->get_parameter("x_deta");
					rclcpp::Parameter int_param2 = this->get_parameter("y_deta");
					rclcpp::Parameter int_param3 = this->get_parameter("z_deta");
					
					x_now += int_param1.as_int();
					y_now += int_param2.as_int();
					z_now += int_param3.as_int();
					
					//cout<< "x:"<<x_now << "y:"<< y_now << "z:" << z_now <<endl;
							
					set_parameter(rclcpp::Parameter("x_deta", 0));
					set_parameter(rclcpp::Parameter("y_deta", 0));
					set_parameter(rclcpp::Parameter("z_deta", 0));
					
					publish_trajectory_setpoint(x_now,y_now,z_now);
					
					//publish_trajectory_setpoint(corrdinate_vect_x.load(),corrdinate_vect_y.load(), corrdinate_vect_z.load());





			    
			    }
			    
			   else if(takeoff_land ==5)//excute task
				{
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					publish_offboard_control_mode();
					publish_trajectory_setpoint(corrdinate_vect_x.load(),corrdinate_vect_y.load(), corrdinate_vect_z.load());


				}

			}


		};
		timer_ = this->create_wall_timer(200ms, timer_callback);


	}
	~OffboardControl()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	}

	void arm() const;
	void disarm() const;

private:

	std::atomic<uint64_t> timestamp_;   //!< common synced timestampe
	std::atomic<float_t> x_coordinate_;   //!< common synced timestamped
	std::atomic<float_t> y_coordinate_;   //!< common synced timestamped
	std::atomic<float_t> z_coordinate_;

	// vector<Corrdinate> m_vecMain;
	vector<uint32_t>test_corrdinate_vect;

	bool arm_able = false;              //lock flag   
	bool reach_position = false; 
	bool takeoff_flag = true;

	float x_init_corrdinate,y_init_corrdinate,z_init_corrdinate; // takeoff  begin position corrdinate       
	uint8_t takeoff_state_;										 // takeoff  state 
	
	signed int x_now = 0,y_now = 0,z_now = 4;
	signed int x_deta = 0,y_deta = 0, z_deta = 0;


	//std::vector<std::double_t> my_double_array;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    uint64_t j;
	int takeoff_land = 0;	//起飞标志位，0：起飞，1：降落，2：返航
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

	rclcpp::Service<uav_interfaces::srv::UavControl>::SharedPtr server_;
	rclcpp::Client<uav_interfaces::srv::UavControl>::SharedPtr client_;
	void uav_control_callback(const uav_interfaces::srv::UavControl::Request::SharedPtr request,
        const uav_interfaces::srv::UavControl::Response::SharedPtr response)
    {
        //client在哪里？？
		//对请求数据进行处理
		//takeoff_land = (float)(request->takeoff_land);
		corrdinate_takeoff_land.store(request->takeoff_land);
		corrdinate_x_deta.store(request->x_deta);
		corrdinate_y_deta.store(request->y_deta);
		corrdinate_z_deta.store(request->z_deta);
		
		//cout<< corrdinate_x_deta << corrdinate_y_deta << corrdinate_z_deta << endl;
		
		response->success = takeoff_land;
		
		set_parameter(rclcpp::Parameter("x_deta", corrdinate_x_deta.load()));
		set_parameter(rclcpp::Parameter("y_deta", corrdinate_y_deta.load()));
		set_parameter(rclcpp::Parameter("z_deta", corrdinate_z_deta.load()));

    }




	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_position_subscription;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr px4_gps_position_subscription_;

	rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr corrdinate_subscription_;

	
	std::vector<std::string> param_names = {"x", "y", "z"};
	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint(float_t x, float_t y ,float_t z) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0,float param7 = 15.0) const;

	void publish_vehicle_command_test(uint32_t command, float_t param1,float_t param2,float_t param3,
													float_t param4,float_t param5,float_t param6,float_t param7) const;

	void desclar_parameter_();
	void get_parameter_(); 
	void publish_takeoff_status(uint8_t takeoff_state_) const;
	void publish_position_setpoint(uint8_t type,float altitude) const;
    void set_home();
	void sub_corrdinate_topic_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
};


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
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
	declare_parameter<int>("takeoff_land", takeoff_land);
	
	declare_parameter<signed int>("x_deta", x_deta);
	declare_parameter<signed int>("y_deta", y_deta);
	declare_parameter<signed int>("z_deta", z_deta);
	
	declare_parameter<std::float_t>("fly_height", fly_height);
	
	declare_parameter("my_double_array", std::vector<std::double_t>{0.0,0.0});//state uav initative position x:0.0,y:0.0
}

void OffboardControl::get_parameter_() 
{
	//get_parameter("takeoff_land", takeoff_land);//fly mode flag: 0.0:takeoff  1.0:land  2.0:return  
	
	//get_parameter("x_deta", x_deta);
	//get_parameter("y_deta", y_deta);
	//get_parameter("z_deta", z_deta);
	
	
	set_parameter(rclcpp::Parameter("x_deta", corrdinate_x_deta.load()));
	set_parameter(rclcpp::Parameter("y_deta", corrdinate_y_deta.load()));
	set_parameter(rclcpp::Parameter("z_deta", corrdinate_z_deta.load()));
	
	cout<< "get_param "<< x_deta << y_deta << z_deta <<endl;
	


	
    	//get_parameter("fly_height", fly_height);//get ros2 sent fly_height parameter 
	//get_parameter("my_double_array",my_double_array);//get ros2 sent fly position axis. x:  y:
}

void OffboardControl::sub_corrdinate_topic_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "订阅到坐标 x:'%d' y:'%d' z:'%d'", msg->data[0],msg->data[1],msg->data[2]);
	this->test_corrdinate_vect.push_back(msg->data[0]);
	this->test_corrdinate_vect.push_back(msg->data[1]);
	this->test_corrdinate_vect.push_back(msg->data[2]);
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
	msg.position = {-x + x_init_corrdinate,-y + y_init_corrdinate,-z + z_init_corrdinate};

	// msg.vx = 1.0;
	// msg.vy = 1.0;
	// msg.vz = 1.0;
	// msg.acceleration = {3000.0,3000.0,3000.0};

	msg.yaw = 0.0; // [-PI:PI]

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

	//rclcpp::spin(std::make_shared<OffboardControl>());
    
	rclcpp::executors::MultiThreadedExecutor executor;
    std::cout << "running hear" << std::endl;
	auto subnode = std::make_shared<DualThreadedNode>(); //sub suanfa obj corrdinate
	auto pubnode = std::make_shared<OffboardControl>();	 // control uav
	auto position_node = std::make_shared<User>();		//sub px4 local position , send it to suanfa node
															
														
	executor.add_node(pubnode);
	executor.add_node(subnode);
	executor.add_node(position_node);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}





