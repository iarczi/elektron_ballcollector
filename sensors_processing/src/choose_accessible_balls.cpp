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

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

#define PI 3.14159265

bool explore;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

costmap_2d::Costmap2DROS* costmap_ros;
costmap_2d::Costmap2D costmap;

void alghoritmStateCallBack(const std_msgs::String& msg);
ros::Subscriber alghoritm_state_sub_;


class ChooseAccessibleBalls {

	ros::NodeHandle nh_;
	ros::Subscriber all_balls;
	ros::Publisher goal_pub_;
	ros::Publisher accesible_balls;
	bool first;
	tf::TransformListener tf_listener_;


public:
	MoveBaseClient ac;

	ChooseAccessibleBalls()  :
		ac("move_base", true){
		all_balls = nh_.subscribe < geometry_msgs::PoseArray > ("/allBallsXYZ", 10, &ChooseAccessibleBalls::allBallsCb, this);
		goal_pub_ = nh_.advertise < move_base_msgs::MoveBaseActionGoal > ("goal", 1);
		accesible_balls = nh_.advertise < geometry_msgs::PoseArray > ("accesible_balls", 1);

//		alghoritm_state_sub_ =  nh_.subscribe < std_msgs::String > ("alghoritm_state", 1, &ChooseAccessibleBalls::alghoritmStateCallBack, this);

		first = false;
		explore = true;
	}

	~ChooseAccessibleBalls() {
	}
	void allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg);

	void publishPose(float x, float y, geometry_msgs::Quaternion qMsg);
	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);
	void transFromCameraToOdomPosition(double x_cam_pose, double y_cam_pose, double z_cam_pose,
			double &x_odom_pose, double &y_odom_pose, double &z_odom_pose, tf::Transform& tfCP);
	void transFromOdomToMapPosition(double x_odom_pose, double y_odom_pose, double theta, double &x_map_pose, double &y_map_pose, tf::Quaternion& q);
	void getRobotPositionInOdom(double &x_odom_pose, double &y_odom_pose);
	bool canMove(float x, float y);
//	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);

	float getAngle(float x1, float y1, float x2, float y2);

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "chooseAccessibleBalls");
	ChooseAccessibleBalls chooseAccessibleBalls;

	tf::TransformListener tf_listener;
	costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_listener);

	ros::NodeHandle nh;
	alghoritm_state_sub_ =   nh.subscribe("alghoritm_state", 1, alghoritmStateCallBack);


	
	//wait for the action server to come up
	  while(!chooseAccessibleBalls.ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	  }


	ros::spin();
	return 0;

}

void ChooseAccessibleBalls::allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg){/
	ros::Time start = ros::Time::now();


	double robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);
	std::vector<geometry_msgs::Pose> allBalls = all_balls_msg->poses;

	geometry_msgs::PoseArray accesibleBallsMsg;
	accesibleBallsMsg.header.frame_id ="/odom";
	std::vector<geometry_msgs::Pose> poses;



	ros::Time past = all_balls_msg->header.stamp;
	tf::StampedTransform tfOC;								//	kamera w odom
	tf_listener_.waitForTransform("/odom", "/openni_rgb_optical_frame", past, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/openni_rgb_optical_frame", past,  tfOC);




	for (unsigned int i = 0; i < allBalls.size(); i++){
	

		double x = allBalls[i].position.x;
		double y = allBalls[i].position.y;
		double z = allBalls[i].position.z;

		double ball_odom_x, ball_odom_y, ball_odom_z;


		transFromCameraToOdomPosition(x, y, z, ball_odom_x, ball_odom_y, ball_odom_z, tfOC);

		if(ball_odom_z < 0.01 ||ball_odom_z > 0.1){
			continue;
		}

		float goal_odom_x, goal_odom_y;

		float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );
		ROS_INFO("dist = %f", dist);


		float dx = ball_odom_x - robot_odom_x;
		float dy = ball_odom_y - robot_odom_y;

		goal_odom_x = robot_odom_x + dx * (dist - .70) / dist;
		goal_odom_y = robot_odom_y + dy * (dist - .70) / dist;


		geometry_msgs::Pose pose;

		pose.position.x = ball_odom_x;
		pose.position.y = ball_odom_y;
		pose.position.z = ball_odom_z;

		pose.orientation = allBalls[i].orientation;

		poses.push_back(pose);


		bool canMove1 = canMove(goal_odom_x, goal_odom_y);

		if(canMove1==true){

			geometry_msgs::Pose pose;

			pose.position.x = ball_odom_x;
			pose.position.y = ball_odom_y;
			pose.position.z = ball_odom_z;

			pose.orientation = allBalls[i].orientation;

			poses.push_back(pose);

		}
		else{
		}


	}

	ros::Time finish = ros::Time::now();
	ros::Duration duration = finish - start;
	double durSec = duration.toNSec()/1000000;
	double avgDur = durSec/allBalls.size();

		accesibleBallsMsg.poses = poses;
		accesible_balls.publish(accesibleBallsMsg);

}

void ChooseAccessibleBalls::publishPose(float x, float y, geometry_msgs::Quaternion qMsg){

	  move_base_msgs::MoveBaseGoal goal;

	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;

	  goal.target_pose.pose.orientation = qMsg;

	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.header.frame_id ="/map";


	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

}

void ChooseAccessibleBalls::setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	tf::Quaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}

/**
 * Metoda przeklada wspolrzedne pileczki w ukladzie kamery do ukladu odom
 */
void ChooseAccessibleBalls::transFromCameraToOdomPosition(double x_cam_pose, double y_cam_pose, double z_cam_pose,
		double &x_robot_pose, double &y_robot_pose, double &z_robot_pose, tf::Transform& tfOC){

	tf::Transform tfCP;
	tfCP.setOrigin(tf::Vector3(x_cam_pose, y_cam_pose, z_cam_pose));			//	pileczka w kamerze


	tf::Transform tfOP = tfOC * tfCP;

	x_robot_pose = tfOP.getOrigin ().x();				//	wspolrzedne pileczki w ukladzie odom
	y_robot_pose = tfOP.getOrigin ().y();				//	wspolrzedne pileczki w ukladzie odom
	z_robot_pose = tfOP.getOrigin ().z();

}

bool ChooseAccessibleBalls::canMove(float x, float y){
	return true;

	
	  // Get a copy of the current costmap to test. (threadsafe)
	  costmap_2d::Costmap2D costmap;
	  if(costmap_ros != NULL)
		  costmap_ros->getCostmapCopy( costmap );

	  // Coordinate transform.
	  unsigned int cell_x, cell_y;
	  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	    return false;
	  }

	  double cost = double( costmap.getCost( cell_x, cell_y ));
	  if(cost <= 127){
		  return true;
	  }
	  else{
		  return false;
	  }
}

void ChooseAccessibleBalls::getRobotPositionInOdom(double &x_odom_pose, double &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}

void ChooseAccessibleBalls::transFromOdomToMapPosition(double x_odom_pose, double y_odom_pose, double theta,
		double &x_map_pose, double &y_map_pose, tf::Quaternion& q){

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


float ChooseAccessibleBalls::getAngle(float x1, float y1, float x2, float y2){
	return atan2(y2-y1, x2-x1);
}


void alghoritmStateCallBack(const std_msgs::String& msg){
	ROS_INFO("alghoritmStateCallBack ");
	std::cout<<msg.data<<std::endl;
	if(msg.data.compare("SEARCH_BALLS") ==0 ){
		ROS_INFO("explore == true");
		explore = true;
	}else if(msg.data.compare("GO_TO_BALL") == 0 ){
		ROS_INFO("explore == false");
		explore = false;
	}
}


