#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>


class MoveRobot{
	ros::NodeHandle nh_;

	ros::Subscriber odom_subscriber_;
	ros::Subscriber request_subscriber_;
	ros::Publisher cmd_vel_publisher_;
	ros::Publisher state_publisher_;				//	// 0 - nic, 1 - jedzie, 2 - dojechal

	geometry_msgs::Point start_position_;
	geometry_msgs::Point last_position_;
	bool first_request_recived;
	float distance_;					//	odleglosc do przejechania w [m]
	geometry_msgs::Twist vel;
	bool goal_done_;

public:
	
	MoveRobot()  {
		odom_subscriber_ = nh_.subscribe ("/odom", 1, &MoveRobot::odomCb, this);
		request_subscriber_ = nh_.subscribe("/robot_go_straight", 1, &MoveRobot::requestCb, this);
		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
		state_publisher_ = nh_.advertise<std_msgs::Int16> ("/robot_go_straight_state", 1);

		vel.angular.z = 0.0;
		vel.linear.x = 0.0;
		
		first_request_recived = false;
		goal_done_ = false;
	}

	~MoveRobot() {
	}

	void odomCb(const nav_msgs::OdometryConstPtr& odometry);
	void requestCb(const std_msgs::Float32& request );
	float getDistanceFromStart();

	void publish();
	void publishStateNothing();
	void publishStateRunning();
	void publishStateDone();

};

int main(int argc, char** argv) {
	//TODO
	ros::init(argc, argv, "move_robot_straight");
	MoveRobot moveRobot;

	ros::Rate loop_rate(100);
	while(ros::ok()){
		moveRobot.publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}
void MoveRobot::publish(){
	cmd_vel_publisher_.publish(vel);
	
}
void MoveRobot::odomCb(const nav_msgs::OdometryConstPtr& odometry){

	ROS_INFO("enter odomCb move robot straight");
	last_position_ = odometry->pose.pose.position;

	if(first_request_recived == false){
//		publishStateNothing();
		return;
	}

	float distFromStart = getDistanceFromStart();
	ROS_INFO("distFromStart = %f", distFromStart);
	ROS_INFO("distance to go = %f", fabs(distance_ - distFromStart));
	if(goal_done_ == false && fabs(distance_ - distFromStart) < 0.05){
		//	stop
		geometry_msgs::Twist vel;
		vel.angular.z = 0;
		vel.linear.x = 0;
		cmd_vel_publisher_.publish(vel);
		goal_done_ = true;
		publishStateDone();
	//	ros::Duration(0.5).sleep();
	}

	if(goal_done_ == true){
		publishStateNothing();
	}else{
		publishStateRunning();
	}
}

void MoveRobot::requestCb(const std_msgs::Float32& request ){
	ROS_INFO("enter requestCb move robot straight");
	start_position_ = last_position_;
	distance_ = request.data;

	ROS_INFO("enter distance_ = %f", distance_);


	//geometry_msgs::Twist vel;
	vel.angular.z = 0;
	if(distance_ > 0){
		vel.linear.x = 0.1;
	}
	else{
		vel.linear.x = -0.1;
		distance_ = -distance_;
	}
	
	cmd_vel_publisher_.publish(vel);

	first_request_recived = true;
	goal_done_ = false;
}


float MoveRobot::getDistanceFromStart(){
	return sqrt(pow(start_position_.x - last_position_.x, 2) + pow(start_position_.y-last_position_.y, 2) );
}


void MoveRobot::publishStateNothing(){
	std_msgs::Int16 message;
	message.data = 0;
	state_publisher_.publish(message);
	ROS_INFO("Nothing");
}

void MoveRobot::publishStateRunning(){
	std_msgs::Int16 message;
	message.data = 1;
	state_publisher_.publish(message);
	ROS_INFO("Running");
}

void MoveRobot::publishStateDone(){
	std_msgs::Int16 message;
	message.data = 2;
	state_publisher_.publish(message);
	ROS_INFO("Done");
}





