#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <actionlib/server/simple_action_server.h>
#include <scheduler/SchedulerAction.h>


#define PI 3.14159265
#define TIME_FOR_GOAL 20.0


typedef enum
{
	STOP = 0,
    FULL_ROTATE = 1,
    RANDOM_ROTATE = 2,
    MAX_FORWARD = 3,
}ExploreState;

typedef enum
{
	USE_MAP = false,
    USE_ROBOT = true,
}GOAL_TYPE;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose2D currPose;

costmap_2d::Costmap2DROS* costmap_ros;
costmap_2d::Costmap2D costmap;


double getAngle(const geometry_msgs::Quaternion& qMsg);
void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);

bool canMove(float x, float y);
double getCost(float x, float y);


class Explore{

private:
	ros::NodeHandle nh_;
	ros::Subscriber alghoritm_state_sub_;
	ros::Subscriber posSub;
	ros::Subscriber deadlock_service_state_sub;


	actionlib::SimpleActionServer<scheduler::SchedulerAction> as_;
	std::string action_name_;
	scheduler::SchedulerFeedback feedback_;
	scheduler::SchedulerResult result_;

	bool explore_;
	ExploreState explore_state_;

public:
	bool firstGoalSend;
	MoveBaseClient ac_;
	tf::TransformListener tf_listener_;


	Explore(std::string name):
		as_(nh_, name, false),
		action_name_(name),
		ac_("move_base", true)
	{
		explore_state_ = STOP;
		explore_ = false;
		firstGoalSend = false;
		
		as_.registerGoalCallback(boost::bind(&Explore::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&Explore::preemptCB, this));

		as_.start();

		alghoritm_state_sub_ =   nh_.subscribe("alghoritm_state", 1, &Explore::alghoritmStateCallBack, this);
		posSub = nh_.subscribe("/amcl_pose", 1, &Explore::poseCallback, this);
		deadlock_service_state_sub = nh_.subscribe("/deadlock_service_state", 1, &Explore::deadlockServiceStateCb, this);


		
	}
	void publishPose(float x, float y, float theta, bool robot);
	void stopExplore();
	void alghoritmStateCallBack(const std_msgs::String& msg);
	void deadlockServiceStateCb(const std_msgs::String& state);
	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom);
	void transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map);

	void transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
			float &x_map_pose, float &y_map_pose, tf::Quaternion& q);

	void getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose);
	void getRobotPositionInMap(float &x_odom_pose, float &y_odom_pose);
	
	float getRobotAngleInMap();
	float getRobotAngleInOdom();
	float addPiToAngle(float angle);

	void randomRotate();
	void randomForward();
	void robotFullRotate();
	void maxForward();

	void executeCB(const scheduler::SchedulerGoalConstPtr &goal);
	void setCanExplore(bool explore){explore_ = explore;};
	ExploreState getExploreState(){ return explore_state_;};
	void setExploreState(ExploreState explore_state){explore_state_ = explore_state;};
	bool isCurrentGoalDone();
	bool isActionServerActive(){return as_.isActive();};


	void goalCB();
	void preemptCB();
};



int main(int argc, char** argv) {

	ros::init(argc, argv, "robot_move");

	Explore robot_explore(ros::this_node::getName());
	
	costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", robot_explore.tf_listener_);

	double infRad = costmap_ros->getInflationRadius();
	ROS_INFO("infRad =%f",infRad);

	 while (!robot_explore.ac_.waitForServer(ros::Duration(5.0))) {
		//ROS_INFO("Waiting for the move_base action server to come up");
	 }

	robot_explore.setExploreState(STOP);

		ros::Rate loop_rate(1);
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
		if (!robot_explore.isActionServerActive()){

			//ROS_INFO("Explore server action isn't active!");

			continue;
		}
		else{
			if(robot_explore.getExploreState() == STOP){
					robot_explore.setExploreState(RANDOM_ROTATE);
					robot_explore.randomRotate();
					ROS_INFO("Start Explore STOP --> RANDOM_ROTATE");
			}
			else if(robot_explore.getExploreState() == RANDOM_ROTATE){

				if(robot_explore.isCurrentGoalDone()){
					robot_explore.setExploreState(MAX_FORWARD);
					robot_explore.maxForward();
					ROS_INFO("go to   RANDOM_ROTATE -->  MAX_FORWARD");
				}
			}
			else if(robot_explore.getExploreState() == MAX_FORWARD){

				if(robot_explore.isCurrentGoalDone()){
					robot_explore.setExploreState(RANDOM_ROTATE);
					robot_explore.randomRotate();
					ROS_INFO("go to   MAX_FORWARD --> RANDOM_ROTATE");
				}
			}
		}		
	

	}
	return 0;
	
}

void Explore::stopExplore(){

	  ac_.cancelAllGoals ();
}



void Explore::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
 // ROS_INFO("I heard pose (x, y) = (%f, %f)", msg->pose.pose.position.x, msg->pose.pose.position.y);

	//	aktualizacja pozycji robota

	currPose.x = msg->pose.pose.position.x;
	currPose.y = msg->pose.pose.position.y;
	currPose.theta = getAngle(msg->pose.pose.orientation);

//	ROS_INFO("currPose.theta= %f", currPose.theta);

}


double getAngle(const geometry_msgs::Quaternion& qMsg){

	  double unused, yaw ;
	  tf::Quaternion q_pose;

	  tf::quaternionMsgToTF(qMsg, q_pose);
	  tf::Matrix3x3 m_pose(q_pose);
	  m_pose.getRPY(unused, unused, yaw);
	  return yaw;
}



void setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	tf::Quaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}

void Explore::publishPose(float x, float y, float theta, bool robot){

	  move_base_msgs::MoveBaseGoal goal;

	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;

	  geometry_msgs::Quaternion qMsg;
	  setAngle(theta, qMsg);

	  goal.target_pose.pose.orientation = qMsg;

	  goal.target_pose.header.stamp = ros::Time::now();
	  if(robot)
	  {
		  goal.target_pose.header.frame_id ="/base_link";
	  } else {
		  goal.target_pose.header.frame_id ="/map";
	  }
	  
	  ROS_INFO("Sending goal...");
	  ac_.sendGoalAndWait(goal,ros::Duration(TIME_FOR_GOAL),ros::Duration(0.1) );
	  firstGoalSend = true;

}



float Explore::addPiToAngle(float angle){
	angle += PI;								//	kąt od 0 do 2PI
	angle += PI;								//	kąt od 0 do 2PI przekręcony o PI

	if(angle > 2*PI){
		angle = angle - 2*PI;
	}
	angle -= PI;
	return angle;
}

void Explore::robotFullRotate(){
	float angle = (360 * PI) / 180.0;
	publishPose(0, 0, angle, USE_ROBOT);
}


void Explore::randomRotate(){

	float angle = 70 + ((360-90+1)*rand()/(RAND_MAX+1.0));
	angle = ((angle) * PI) / 180.0;
	publishPose(0, 0, angle, USE_ROBOT);

}



void Explore::randomForward(){

	double forward = 0.5 + ((2.0-0.3+1)*rand()/RAND_MAX+1.0);
	ROS_INFO("forward %f", forward);
	publishPose(forward, 0, 0, USE_ROBOT);
	
}



void Explore::maxForward(){

	float d_x = 0.1, d_y=0.0, x_map, y_map;
	transfromRobotToMapPosition(d_x, d_y, x_map, y_map);
		
	 while (canMove(x_map, y_map)){

		d_x += 0.1;
		transfromRobotToMapPosition(d_x, d_y, x_map, y_map);

	}; 
	ROS_INFO("d_x = %f", d_x);

	publishPose(x_map, y_map, get, getRobotAngleInMap(), USE_MAP);

}



void Explore::getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}
void Explore::getRobotPositionInMap(float &x_odom_pose, float &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}


float Explore::getRobotAngleInMap(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/base_link", now,  tfOR);

//	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
//	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

	return tf::getYaw(tfOR.getRotation());

}


float Explore::getRobotAngleInOdom(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	return tf::getYaw(tfOR.getRotation());

}


bool canMove(float x, float y){
	  // Get a copy of the current costmap to test. (threadsafe)
	  costmap_2d::Costmap2D costmap;
	  costmap_ros->getCostmapCopy( costmap );

	  // Coordinate transform.
	  unsigned int cell_x, cell_y;
	  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	
			ROS_INFO("!costmap x,y =%f, %f, cell_x,y = %i, %i ", x, y, cell_x, cell_y);
	 	return false;
	  }

	  double cost = double( costmap.getCost( cell_x, cell_y ));
	 // ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
	 
	  if(cost <= 1){
		  return true;
	  }
	  else{
		 return false;
	  }
	 
}

double getCost(float x, float y){
  // Get a copy of the current costmap to test. (threadsafe)
  costmap_2d::Costmap2D costmap;
  costmap_ros->getCostmapCopy( costmap );

  // Coordinate transform.
  unsigned int cell_x, cell_y;
  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	  return -1.0;
  }

  double cost = double( costmap.getCost( cell_x, cell_y ));
//  ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
  return cost;
}


void Explore::transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom){

// 	ROS_INFO(" enter transfromRobotToOdomPosition  robot pose (%f, %f)", x_robot, y_robot);

	ros::Time now = ros::Time::now();

//	ROS_INFO("1");
	tf::StampedTransform tfRO;
//	ROS_INFO("2");
	tf_listener_.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
//	ROS_INFO("3");
	tf_listener_.lookupTransform ("/base_link", "/odom",  now,  tfRO);
//	ROS_INFO("4");
	tf::Transform tfOR = tfRO.inverse();
//	ROS_INFO("5");
	float noused = 0.0;
//	ROS_INFO("6");
	tf::Transform tfRD;
//	ROS_INFO("7");
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));
//	ROS_INFO("8");
	tf::Transform tfOD = tfOR * tfRD;


	 x_odom = tfOD.getOrigin ()[0];				//	wspolrzedna x_robot w ukladzie odom
	 y_odom = tfOD.getOrigin ()[1];				//	wspolrzedna y_robot w ukladzie odom

	// ROS_INFO("ROBOT TO ODOM x_odom = %f,  y_odom = %f", x_odom, y_odom);
}

void Explore::transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map){

	ros::Time now = ros::Time::now();

	tf::StampedTransform tfRO;
	tf_listener_.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/base_link", "/odom", now,  tfRO);
	tf::Transform tfOR = tfRO.inverse();


	tf::StampedTransform tfOM;
	tf_listener_.waitForTransform("/odom", "/map", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/map", now,  tfOM);
	tf::Transform tfMO = tfOM.inverse();


	//tf::Transform tfRM = tfMO*tfOR;

	float noused = 0.0;

	tf::Transform tfRD;
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));

	tf::Transform tfOD = tfOR * tfRD;


	tf::Transform tfMD = tfMO * tfOD;


	x_map = tfMD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie mapy
	y_map = tfMD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie mapy

	//ROS_INFO("ROBOT TO MAP x_robot = %f,  y_robot = %f, x_map = %f,  y_map = %f", x_robot, y_robot, x_map, y_map);
}


void Explore::transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
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
	
	//ROS_INFO("ODOM TO MAP x_odom = %f,  y_odom = %f, x_map = %f,  y_map = %f", x_odom_pose, y_odom_pose, x_map_pose, y_map_pose);
}


bool Explore::isCurrentGoalDone(){
	bool is_done = ac_.getState().isDone();
	return is_done;
}



//	metoda jest uruchamiana w osobnym watku i nie blokuje calbackow
void Explore::executeCB(const scheduler::SchedulerGoalConstPtr &goal){
	ROS_INFO("enter executeCB, goal = %i", goal->value);

	if(goal->value == 1){
		explore_ = true;

	}
	else if(goal->value == 0){
		//	koniec ekslporacji
		stopExplore();
		explore_state_ = STOP;
		explore_ = false;
	}

	feedback_.value = 0;

	as_.publishFeedback(feedback_);
	result_.value = feedback_.value;

	as_.setSucceeded(result_);
	ROS_INFO("leave executeCB");

}


void Explore::goalCB(){

	explore_state_ = FULL_ROTATE;
	as_.acceptNewGoal();
  }

void Explore::preemptCB(){
	ROS_INFO("%s: Preempted", action_name_.c_str());
	explore_ = false;
	explore_state_ = STOP;
	ac_.cancelAllGoals ();
	as_.setPreempted(); 
}








