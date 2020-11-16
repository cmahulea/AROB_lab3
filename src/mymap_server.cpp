#include <ros/ros.h>
#include <AROB_lab3/arob_lab3.h>
#include <string>
#include <iostream>
#include <std_msgs/UInt16MultiArray.h>
#include <iostream>
#include <stdio.h> 
#include <stdlib.h> 
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>



	std::vector < std::vector<int> > read_pgm_image(std::string filnam, char filemod[], char  comment[], int & R, int & C, int & bits)     /*reads PGM images*/
	{
	using namespace std;

	    vector < vector<int> > img;
	    ifstream ifs;
	    ifstream iftest;
	    string newlin;
	    iftest.open(filnam,ios::in);
	    if(iftest.fail()){
                cout << "File not exists " << endl;//File does not exist code here
		R = 0; C = 0;
		iftest.close();
		return img;
            }
	    int header_size = 3;
    	    for (int i = 0; i < 4; i++){
    		getline(iftest,newlin);
    		if(newlin.data()[0] == '#'){
		        header_size = 4;
		        break;
    		}
    	     }
             iftest.close();
    	     ifs.open(filnam, ios::in);
    	     bool h = 0;
             int i = 0;
	     for (int i = 0; i < header_size; i++){
	        streampos old = ifs.tellg();
        	getline(ifs,newlin);
                int N1;
        	stringstream t(newlin);
        	t >> N1;
        	if(strcmp(newlin.data(),"P2") == 0 | strcmp(newlin.data(),"P5") == 0 ){
	            strcpy(filemod,newlin.data());
        	}
        	else if(newlin.data()[0] == '#'){
	            strcpy(comment,newlin.data());
        	}
        	else if(t){
	            int NN;
	            t >> NN;
	            if (t){
	                C = N1;
        	        R = NN;
        	    }
        	    else
        	    {
        	        bits = N1;
        	    }
        	}
        	else{
	            cout << "\n Overread into bits!! Going back to bit line!!! \n";
        	    ifs.seekg(old);
        	    break;
	        }        
	    }
	    ifs.close();    
	    ifstream newp;
	    newp.open(filnam, ios::in|ios::binary); 	   
	    char tt;
	    newp >> tt;    
	    for (int i = 0; i < header_size; i++){
	        getline(newp,newlin);
    	    }    
/*	cout << filemod << " : Filetype <-  \n";
	cout << comment << " : Comment <-  \n";
	cout << R << " : R - Height <-  \n";
	cout << C<< " : C - Width <-  \n";
	cout << bits<< " : Maximum Value <-  \n";*/
	vector <int> zvc(C);
	fill(zvc.begin(),zvc.end(),0);
	for(int i = 0; i < R; i++){
	    img.push_back(zvc);
	}
	int k = 0,l = 0;
	long cnt = 0;
	for(k = 0; k < R; k++){
        for(l = 0; l < C; l++){
            if(newp.eof() == 1){
	            break;    
            }            
        int val = 0;
        int p = 7;
        char c11;
        newp.get(c11);        
        while(p >=0){            
            val += (((c11 >> p) & 1) ) * pow(2,p);
            p--;
        }        
        img[k][l] = val;        
        cnt++;        
        }
    }
    newp.close();
    return img;
    }//read_pgm_image



bool partition(AROB_lab3::arob_lab3::Request  &req, 
		AROB_lab3::arob_lab3::Response &res )
{
 	std_msgs::UInt16MultiArray grid;
	//Clear array
	 grid.data.clear();
	char filt[100];                                        
	char comment[1000] = " ";
	int R,C,bits;
	std::vector < std::vector <int> > img;
	img  = read_pgm_image(req.file_name,filt,comment,R,C,bits);   
	res.cols = round(C/req.size);
	res.rows = round(R/req.size);
	for(int i = 0; i < res.rows; i++){
		for(int j = 0; j < res.cols; j++){
			bool isfree = true;
			for(int ii = 0; ii < req.size; ii++){
				for(int jj = 0; jj < req.size; jj++){
					if (img[i*req.size+ii][j*req.size+jj] < bits*req.threshold){
					isfree = false;
					break;}
				}
			}
			if (!isfree) grid.data.push_back(0);
			else grid.data.push_back(bits);
		}
	}


  res.grid = grid.data;

  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mymap_server");
  ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
  ros::ServiceServer service = n.advertiseService("mymap", partition);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}

