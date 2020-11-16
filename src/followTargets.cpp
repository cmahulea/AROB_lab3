#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <stdint.h>


using namespace std;

class FollowTargetsClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber pub_plan_;
	ros::Subscriber scan_sub_;
	ros::Publisher recompute_path_new_init_;
	geometry_msgs::PoseStamped Goal;
        //ifstream inFile;
	std::vector<std::vector<float> > targets;
	bool obstacle;
	bool recompute = false;

public:
	FollowTargetsClass() {
		// Subscribe to input video feed and publish output video feed
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &FollowTargetsClass::positionChange, this);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		recompute_path_new_init_ = nh_.advertise<geometry_msgs::PoseStamped>("recompute_path_new_init", 1);
		pub_plan_ = nh_.subscribe("/multi_path", 1, &FollowTargetsClass::getPath, this);
		scan_sub_ = nh_.subscribe("/base_scan", 10, &FollowTargetsClass::checkObs, this);	
		obstacle = false;
	}

	~FollowTargetsClass() {}
	
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

	void printTargets(){
		// Displaying the 2D vector of targets
		for (int i = 0; i < targets.size(); i++) { 
	       		cout << "Target " << i << ": (" << targets[i][0] << "," << targets[i][1] << ")" << endl; 
		} 
		
	}
	void getPath(const nav_msgs::Path& path) {


		//FOR YOU

		this->printTargets();
	}

	void positionChange(const nav_msgs::Odometry& msg) {

		//FOR YOU
	}
};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

