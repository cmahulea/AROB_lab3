#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <stdio.h> 
#include <stdlib.h> 
#include <math.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;


class Lowlevelcontrol {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;
	ros::Subscriber scan_sub_;
	geometry_msgs::PoseStamped Goal;

	float krho, kalpha, kbeta;
	bool obstacle;
public:
	Lowlevelcontrol(int x0, int y0) {
		// Subscribe to input video feed and publish output video feed
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &Lowlevelcontrol::positionCb, this);
		goal_sub_ = nh_.subscribe("goal", 1, &Lowlevelcontrol::goalCb, this);
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		scan_sub_ = nh_.subscribe("/base_scan", 10, &Lowlevelcontrol::checkObs, this);			
		Goal.pose.position.x = x0;
		Goal.pose.position.y = y0;
		kalpha = 2;
		krho = 1;
		kbeta = 0;
		obstacle = false;
	}

	~Lowlevelcontrol() {
	}

	void checkObs(const sensor_msgs::LaserScan::ConstPtr& scan) {

		int no_ranges = scan->ranges.size();
		obstacle = false;
		for(int i=int(no_ranges/2-no_ranges/10);i<int(no_ranges/2+no_ranges/10);i++){
			if(scan->ranges[i] < 0.4){
				obstacle=true;
				cout << "Obstacle at:" << scan->ranges[i] << " meters." << endl;
				return;
			}
		}
	}

	void goalCb(const geometry_msgs::PoseStamped& msg) {

		std::cout << " Goal Update: "<< msg.pose.position.x << endl;
		std::cout << " Goal Update: "<< msg.pose.position.y << endl;
		Goal = msg;
	}

	void positionCb(const nav_msgs::Odometry& msg) {
		
		if(obstacle) return;
		geometry_msgs::Twist input;
		
		float ex = msg.pose.pose.position.x - Goal.pose.position.x;
		float ey = msg.pose.pose.position.y - Goal.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);
		float beta = atan2(ey,ex) + M_PI;
		if (beta < -M_PI)
			beta = 2*M_PI - abs(beta);
		if (beta > M_PI)
			beta = -2*M_PI + beta;
		float alpha = beta - tf::getYaw(msg.pose.pose.orientation);
		std::cout << "ex: "<< ex << "; ";
		std::cout << "ey: "<< ey << "; ";
/*		std::cout << "Qx: "<< msg.pose.pose.orientation.x << " ";
		std::cout << "Qy: "<< msg.pose.pose.orientation.y << " ";
		std::cout << "Qz: "<< msg.pose.pose.orientation.z << " ";
		std::cout << "Qw: "<< msg.pose.pose.orientation.w << " ";

		std::cout << "Rho: "<< rho << " ";
		std::cout << "Alpha: "<< alpha << " ";
		std::cout << "Beta: "<< beta << endl;
*/
		std::cout << "X: "<< msg.pose.pose.position.x << " ";
		std::cout << "Y: "<< msg.pose.pose.position.y << " ";
		std::cout << "Th: "<< tf::getYaw(msg.pose.pose.orientation) << endl;

		if(((alpha>M_PI/4) || (alpha<-M_PI/4)) && (rho > 0.5)){
			input.linear.x = 0;
			input.angular.z = 2;
			std::cout << "alpha: "<< alpha << " ";
			std::cout << "V: "<< input.linear.x << " ";
			std::cout << "W: "<< input.angular.z << endl << "ROTATING" << endl;
			velocity_pub_.publish(input);
			return;
		}

		if (rho > 0.3){
			input.linear.x = krho*rho;
			input.angular.z = kbeta*beta + kalpha*alpha;
		}
		else
		{
			input.linear.x = 0;
			input.angular.z = 0;
		}
		std::cout << "V: "<< input.linear.x << " ";
		std::cout << "W: "<< input.angular.z << endl;
		velocity_pub_.publish(input);
	}

	
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	int x0,y0;
	nh.getParam("/x_0", x0);
	nh.getParam("/y_0", y0);
	cout << "Initial position of the robot: (" << x0 << "," << y0 << ")" << endl;
	Lowlevelcontrol llc(x0,y0);

	ros::spin();
	return 0;
}
