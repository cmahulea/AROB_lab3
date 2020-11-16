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
#include <string>
#include <vector>
#include <sstream>
#include <tf/transform_listener.h>
#include <AROB_lab3/arob_lab3.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include "TreeNode.cpp"
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

using namespace std;


class Highlevelcontrol {
	ros::NodeHandle nh_;
	ros::ServiceClient client;
	ros::Publisher pub_plan;
	ros::Subscriber recompute_path_new_init_;
	ros::Subscriber recompute_path_new_target_;
	vector< vector <int> > grid;
	std::vector <int> init_point;
	std::vector <int> target_point;
	TreeNode *itr;
public:
	Highlevelcontrol() {
		//init_point = transform(init_point_);
		//target_point = transform(target_point_);
		pub_plan = nh_.advertise<nav_msgs::Path>("multi_path",1);
		recompute_path_new_init_ = nh_.subscribe("recompute_path_new_init", 1, &Highlevelcontrol::computePath_new_init, this);
		recompute_path_new_target_ = nh_.subscribe("recompute_path_new_target", 1, &Highlevelcontrol::computePath_new_target, this);
		geometry_msgs::PoseStamped target_point2,init_point2;
		nh_.getParam("x_0", init_point2.pose.position.x);
		nh_.getParam("y_0", init_point2.pose.position.y);
		nh_.getParam("x_f", target_point2.pose.position.x);
		nh_.getParam("y_f", target_point2.pose.position.y);

		init_point = transform(init_point2);
		target_point = transform(target_point2);
		cout << "Initial point: (" << init_point[0] << "," << init_point[1] << ");" << endl;
		cout << "Target point: (" << target_point[0] << "," << target_point[1] << ");" << endl;
		
		AROB_lab3::arob_lab3 srv;
		double size,threshold;
		string file_name;
		nh_.getParam("pgm_file", file_name);
		nh_.getParam("size", size);
		nh_.getParam("threshold", threshold);


		ros::ServiceClient client = nh_.serviceClient<AROB_lab3::arob_lab3>("mymap");
		srv.request.file_name = file_name;
		srv.request.size = size;
		srv.request.threshold = threshold;

		std::cout << "File: " << file_name << "; size=" << size << "; threshold=" << threshold << ";" << endl;
		if (client.call(srv))
		{
			int rows = (int)srv.response.rows;
			int cols = (int)srv.response.cols;
			ROS_INFO("Rows: %d cols: %d",rows,cols);
			int index = 0;
			for(int i=0; i<rows;i++){
				vector<int> vect; 
				for(int j = 0;j<cols;j++){
					vect.push_back((int)srv.response.grid[index++]);
				}
				grid.push_back(vect);
				vect.clear();
			}
		//print the partition
		cout << "C=[" << endl; 
		for (int i = 0; i < grid.size(); i++) { 
			for (int j = 0; j < grid[i].size()-1; j++){ 
				cout << grid[i][j] << ","; 
			}
			cout << grid[i][grid[i].size()-1] << ";" << endl; 
		} 
		cout << "];" << endl; 
		}
		else {
		    ROS_ERROR("Failed to call service mymap to partition the environment!");
  		}


//		cin.get();
		itr = new TreeNode(init_point);
		//itr->printNode();
	}
	~Highlevelcontrol() {
	}

	void computePath_new_target(const geometry_msgs::PoseStamped& msg) {
		target_point = transform(msg);
		std::cout << " High level Goal Update: ("<< msg.pose.position.x << "," << msg.pose.position.y << ")" << endl;
		std::cout << " High level Goal Update (Matlab): ("<< target_point[0] << "," << target_point[1] << ")" << endl;
		itr = new TreeNode(init_point);
		itr->printNode();
//		cin.get();
		this->RRT();
	}


	void computePath_new_init(const geometry_msgs::PoseStamped& msg) {
		init_point = transform(msg);
		std::cout << " High Level Initial Update: ("<< msg.pose.position.x << "," << msg.pose.position.y << ")" << endl;
		std::cout << " High Level Initial Update (Matlab): ("<< init_point[0] << "," << init_point[1] << ")" << endl;
		itr = new TreeNode(init_point);
		itr->printNode();
//		cin.get();
		this->RRT();
	}


	bool RRT(){
	bool stop = false;
	time_t tnow = time(0);
	srand (tnow);
	int num_generated=0;
	vector <vector <int>> solRRT;
	cout << "Initial node: (" << init_point[0] << "," << init_point[1] << ");" << endl;
	do{


	//FOR YOU

	}//do
	while(!stop);

	//publish the message

	
	//FOR YOU

 
	}//end Highlevelcontrol::RRT

	
	vector <int> transform(const geometry_msgs::PoseStamped& point){ //transform from stage coordinate to grid ones
		vector <int> transf;
		transf.push_back(-point.pose.position.y);
		transf.push_back(point.pose.position.x);
//		cout << "transf[0]=" << transf[0] << "; transf[1]=" << transf[1] << ";" << endl;
		return transf;
	}

	vector <int> invtransform(const geometry_msgs::PoseStamped& point){ //transform from grid matrix to stage coordinate
		vector <int> transf;
		transf.push_back(point.pose.position.y);
		transf.push_back(-point.pose.position.x);
//		cout << "transf[0]=" << transf[0] << "; transf[1]=" << transf[1] << ";" << endl;
		return transf;
	}

	bool obstacleFree(std::vector <int> point1, std::vector <int> point2){

	double alpha;
	std::vector <int> lin_point;
	
	for(alpha=0; alpha<1; alpha=alpha+0.001){
		lin_point.push_back(int(alpha*point1[0] + (1-alpha)*point2[0]));
		lin_point.push_back(int(alpha*point1[1] + (1-alpha)*point2[1]));
		if (grid[lin_point[0]][lin_point[1]] == 0)
			return false;
		lin_point.clear();
	}

	return true;	
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "highcontrol");
	ros::NodeHandle nh("");

  Highlevelcontrol hlc;
//  hlc.RRT();

  ros::spin();
  return 0;
}
