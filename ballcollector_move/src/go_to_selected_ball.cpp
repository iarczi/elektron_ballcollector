#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>


#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>


#include <actionlib/server/simple_action_server.h>
#include <scheduler/SchedulerAction.h>


typedef enum
{
    GO_TO_BALL_WITH_NAV = 0,
    GO_FORWARD_INTRO = 1,
    GO_FORWARD_COLLECT = 2,
    GO_FORWARD_FINISH = 3,
    LOOKING_FOR_BALLS = 4,
    GO_TO_BALL = 5,
    IDLE = 6,


    STOP =7,
    FIRST_STEP_COLLECT = 8,
    SECOND_STEP_COLLECT = 9,
}State;




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoToSelectedBall{

private:
	ros::NodeHandle nh_;
	ros::Subscriber selected_ball_sub_;
	ros::Subscriber go_forward_robot_state_sub_;
	ros::Subscriber deadlock_service_state_sub;

	geometry_msgs::Point current_pose_;
	ros::Publisher hoover_state_pub_;
	ros::Publisher alghoritm_state_pub_;

	ros::Publisher go_forward_robot_pub_;


	tf::TransformListener tf_listener_;

	actionlib::SimpleActionServer<scheduler::SchedulerAction> as_;
	std::string action_name_;

	State state_;

public:
	bool firstGoalSent;
	MoveBaseClient ac;
	bool isBallPoseSet;
	bool is_deadlock_service_run;

	int moveStraightState;			// 0 - nic, 1 - jedzie, 2 - dojechal
	bool moveStraightStateChange;


	void selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose);
	void robotGoStraightStateCb(const std_msgs::Int16& state);
	void deadlockServiceStateCb(const std_msgs::String& state);

	scheduler::SchedulerFeedback feedback_;
	scheduler::SchedulerResult result_;

	GoToSelectedBall(std::string name) :
		as_(nh_, name, boost::bind(&GoToSelectedBall::executeCB, this, _1), false),
		action_name_(name),
		ac("move_base", true)
	{
		selected_ball_sub_ = nh_.subscribe < geometry_msgs::Point > ("/one_selected_ball", 1, &GoToSelectedBall::selectedBallCb, this);
		hoover_state_pub_ = nh_.advertise<std_msgs::Int16> ("hoover_state",1);
		go_forward_robot_pub_ = nh_.advertise< std_msgs::Float32>("/robot_go_straight",1);
		go_forward_robot_state_sub_ = nh_.subscribe("/robot_go_straight_state", 1, &GoToSelectedBall::robotGoStraightStateCb, this);
		alghoritm_state_pub_ = nh_.advertise<std_msgs::String> ("/alghoritm_state",1);
		deadlock_service_state_sub = nh_.subscribe("/deadlock_service_state", 1, &GoToSelectedBall::deadlockServiceStateCb, this);


		firstGoalSent = false;
		isBallPoseSet = false;
		moveStraightState = 0;
		moveStraightStateChange = false;
		is_deadlock_service_run = false;

		as_.start();
		state_ = STOP;

	}
	~GoToSelectedBall() {

	}

	bool isActionServerActive(){return as_.isActive();};
	
	geometry_msgs::Point getCurrentPose();
	float getAngle(float x1, float y1, float x2, float y2);
	void getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose);
	float getRobotAngleInOdom();
	double getRobotAngleInMap();
	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);
	void transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
			float &x_map_pose, float &y_map_pose, tf::Quaternion& q);
	void publishPose(float x, float y);
	void publishPose(float dist_from_ball);
	void publishAngle();
	float getDistanceFromSelectedBall();

	float getAngleDiff();

	void onHoover();
	void offHoover();

	void goForward(float dist);

	void goToBall();

	State getState(){return state_;};
	void setState(State state){state_ = state;};

	void executeCB(const scheduler::SchedulerGoalConstPtr &goal);

};



int main(int argc, char** argv) {
	ros::init(argc, argv, "goToSelectedBall");
	GoToSelectedBall goToSelectedBall(ros::this_node::getName());

	 while(!goToSelectedBall.ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	 }


	ros::Rate loop_rate(1);

	int speed = 0;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		if( goToSelectedBall.getState() == STOP ){
		//		ROS_INFO("STOP state");
				continue;
			}
			else if(goToSelectedBall.getState() == FIRST_STEP_COLLECT){
			//	scheduler zezwolil na jazde, ale node nie ma wspolrzednych pileczki
				if(goToSelectedBall.isBallPoseSet == false){
					ROS_INFO("FIRST_STEP_COLLECT - no ball visible");
					continue;
				}
				else{
					if(goToSelectedBall.getDistanceFromSelectedBall() > 0.6){

					ROS_INFO("FIRST_STEP_COLLECT - go to ball");
					float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
					if(angleDiffRobotGoal > 2.0){						
			
						goToSelectedBall.publishAngle();
						goToSelectedBall.ac.waitForResult();
						goToSelectedBall.goForward(0.1);
					}
					else{
						goToSelectedBall.goForward(0.1);
					}
				}
					else{
						ROS_INFO("FIRST_STEP_COLLECT - ball too close");
					}
				}
			}

	}
	
}
void GoToSelectedBall::setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	tf::Quaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}

float GoToSelectedBall::getDistanceFromSelectedBall(){

	float robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	ball_odom_x = current_pose_.x;
	ball_odom_y = current_pose_.y;
//	ROS_INFO("robot (x, y) i odom = (%f, %f)", robot_odom_x, robot_odom_y);
//	ROS_INFO("ball (x, y) i odom = (%f, %f)", ball_odom_x, ball_odom_y);


	float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );

	return dist;
}


void GoToSelectedBall::selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose){

//	ROS_INFO("selectedBallCb");
	if(selectedBallPose->x == 1000 && selectedBallPose->y == 1000 && selectedBallPose->z == 1000 ){
		isBallPoseSet = false;
//		ROS_INFO("ret 1");
		return;
	}

	current_pose_.x = selectedBallPose->x;
	current_pose_.y = selectedBallPose->y;
	isBallPoseSet = true;

	float angleDiffRobotGoal = getAngleDiff()*180/(3.14);


}


geometry_msgs::Point GoToSelectedBall::getCurrentPose(){
	return current_pose_;
}

void GoToSelectedBall::deadlockServiceStateCb(const std_msgs::String& state){
	if(state.data == "RUNNING"){
		is_deadlock_service_run = true;
	}
	else if(state.data == "NOT_RUNNING"){
		is_deadlock_service_run = false;
	}

}

void GoToSelectedBall::publishPose(float x, float y){
	
	move_base_msgs::MoveBaseGoal goal;



	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	
	geometry_msgs::Quaternion qMsg;
	setAngle(getRobotAngleInMap(), qMsg);
	
	
	
	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.header.frame_id ="/odom";

	ROS_INFO("Sending goal...");
	ac.sendGoal(goal);
	firstGoalSent = true;

}
void GoToSelectedBall::goToBall(){

	
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = getCurrentPose().x;
	goal.target_pose.pose.position.y = getCurrentPose().y;
	
	geometry_msgs::Quaternion qMsg;
	setAngle(getRobotAngleInMap(), qMsg);
	
	
	
	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/odom";

	ROS_INFO("Sending goal...");
	ac.sendGoalAndWait(goal,ros::Duration(120),ros::Duration(0.1));
	firstGoalSent = true;

}


void GoToSelectedBall::publishPose(float dist_from_ball){

	ROS_INFO("enter publishPose, distance = %f", dist_from_ball);

	if(isBallPoseSet== false){
		ROS_INFO("publishPose, wait for ball position, return");
		return;
	}

	float ball_odom_x = getCurrentPose().x;
	float ball_odom_y = getCurrentPose().y;


	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);


	float dx = ball_odom_x - robot_odom_x;
	float dy = ball_odom_y - robot_odom_y;

	float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );

	ROS_INFO("dist = %f",dist);

	float goal_odom_x = robot_odom_x + dx * (dist - dist_from_ball) / dist;
	float goal_odom_y = robot_odom_y + dy * (dist - dist_from_ball) / dist;

	float angle = getAngle(robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y);

	tf::Quaternion q;
	float goal_map_x, goal_map_y;
	transFromOdomToMapPosition(goal_odom_x, goal_odom_y, angle, goal_map_x, goal_map_y, q);



	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q, qMsg);


	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";


	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	firstGoalSent = true;

}


void GoToSelectedBall::publishAngle(){

	ROS_INFO("enter publishAngle");

	if(isBallPoseSet== false){
		ROS_INFO("publishAngle, wait for ball position, return");
		return;
	}


	float ball_odom_x = getCurrentPose().x;
	float ball_odom_y = getCurrentPose().y;

	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	float goal_odom_x = robot_odom_x;// + dx * (dist - dist_from_ball) / dist;
	float goal_odom_y = robot_odom_y;// + dy * (dist - dist_from_ball) / dist;

	float angle = getAngle(robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y);

//tf::Quaternion q;
//float goal_map_x, goal_map_y;
//transFromOdomToMapPosition(goal_odom_x, goal_odom_y, angle, goal_map_x, goal_map_y, q);

	geometry_msgs::Quaternion qMsg;
//	tf::quaternionTFToMsg(q, qMsg);
	setAngle(angle, qMsg);

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_odom_x;
	goal.target_pose.pose.position.y = goal_odom_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/odom";


	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	firstGoalSent = true;
//	isBallPoseSet = false;

	ROS_INFO("return publishAngle");
}





float GoToSelectedBall::getAngle(float x1, float y1, float x2, float y2){
	return atan2(y2-y1, x2-x1);
}


void GoToSelectedBall::getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}


void GoToSelectedBall::transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
		float &x_map_pose, float &y_map_pose, tf::Quaternion& q){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfMO;								//	odom w map
	tf_listener_.waitForTransform("/map", "/odom", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/odom", now,  tfMO);


	tf::Transform tfOD;

	tf::Quaternion quat;
	quat.setRPY(0, 0, theta);

	tfOD.setOrigin(tf::Vector3(x_odom_pose, y_odom_pose, 0.0));			//	docelowe w odom
	tfOD.setRotation( quat );

	tf::Transform tfMD = tfMO * tfOD;							//	docelowe w map


	x_map_pose = tfMD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie map
	y_map_pose = tfMD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie map
	q = tfMD.getRotation();
}



void GoToSelectedBall::onHoover() {
	std_msgs::Int16 state;
	state.data = 0;
	hoover_state_pub_.publish(state);

}

void GoToSelectedBall::offHoover() {
	std_msgs::Int16 state;
	state.data = 1;
	hoover_state_pub_.publish(state);

}


float GoToSelectedBall::getAngleDiff(){

	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	float goalAngle = getAngle(robot_odom_x, robot_odom_y, getCurrentPose().x, getCurrentPose().y);
	float robotAngle = getRobotAngleInOdom();

	return fabs(goalAngle-robotAngle);

}

float GoToSelectedBall::getRobotAngleInOdom(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);
	return tf::getYaw(tfOR.getRotation());

}
double GoToSelectedBall::getRobotAngleInMap(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/base_link", now,  tfOR);


	return tf::getYaw(tfOR.getRotation());


}

void GoToSelectedBall::goForward(float dist){
	std_msgs::Float32 go;
	go.data = dist;
	go_forward_robot_pub_.publish(go);
}


void GoToSelectedBall::robotGoStraightStateCb(const std_msgs::Int16& state){
//	ROS_INFO("enter robotGoStraightStateCb state = %d", state.data);

	if(moveStraightState != state.data){
		moveStraightStateChange = true;
	}
	else{
		moveStraightStateChange = false;
	}
	moveStraightState = state.data;
}



void GoToSelectedBall::executeCB(const scheduler::SchedulerGoalConstPtr &goal){
	ROS_INFO("enter executeCB, goal = %i", goal->value);

	if(goal->value == 0){
		state_ = STOP;
	}
	else if(goal->value == 1){
		state_ = FIRST_STEP_COLLECT;

	}
	else if(goal->value == 2){
		// TODO: sprawdza, czy jest ustawiona pozycja pileczki, albo przesylac ja razem z goalem
		// TODO: dopisac serwer do jazdy do przodu a nie na sleep tak jak teraz
		state_ = SECOND_STEP_COLLECT;
		goForward(0);
		ROS_INFO("enter SECOND_STEP_COLLECT");
		publishAngle();
		ac.waitForResult();


		float dist = getDistanceFromSelectedBall();
		onHoover();
		goForward(dist -0.3);

		ros::Duration(4.0).sleep();

		goForward(-(dist-0.3));
		ros::Duration(5.0).sleep();

		goForward(0);
		
		offHoover();
		ROS_INFO("leave SECOND_STEP_COLLECT");
	}


	as_.publishFeedback(feedback_);
	result_.value = feedback_.value;

	as_.setSucceeded(result_);
	ROS_INFO("leave executeCB");
}





