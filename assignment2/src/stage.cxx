
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cpptk.h"
#include <iostream>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>

using namespace Tk;
using namespace std;


int arraywidth=820;
int arrayheight=700;
bool cells[820][700];

int state = 0;

struct Position {
	/*Coin(int x, int y)
	{
		posx = x;
		poxy = y;
	}*/
	int x;
	int y;
};

struct Vertex{
		Vertex(int posx, int posy){
			x=posx;
			y=posy;
		}
		int x;
		int y;
};
struct Egdes{
	Egdes(Vertex v,Position p){
		one = v;
		two = p;
	}
	Vertex one;
	Vertex two;
};

void operator=(Vertex a, Position b){
	a.x = b.x;
	a.y = b.y;
}

void operator=(Vertex a, Vertex b){
	a.x = b.x;
	a.y = b.y;
}

bool operator<(Position a, Vertex b){
	return sqrt(pow(a.x-b.x)+pow(a.y-b.y));
}

float distance(Position a, Vertex b){
	return sqrt(pow(a.x-b.x)+pow(a.y-b.y));
}

struct Graph{
	//int delta=20;
	int arraywidth=820;
	int arrayheight=700;
	bool cells[820][700];
	vector<Position> edgs;
	vector<Vertex> vertices;
}g;

void near_vertex(Position p){
	//float min=820*720;
	for(int i=0;i<g.vertices.size();++i){
		if(distance(p,))
	}
}

void new_conf(){


}


/*PSEUDO

Algorithm BuildRRT
  Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq)
  Output: RRT graph G

  G.init(qinit)
  for k = 1 to K
    qrand ← RAND_CONF()
    qnear ← NEAREST_VERTEX(qrand, G)
    qnew ← NEW_CONF(qnear, qrand, Δq)
    G.add_vertex(qnew)
    G.add_edge(qnear, qnew)
  return G


*/

void RRT(){

}


void setCell(int i,int j,bool state){
	::cells[i][j]=state;
	if(state){
		".c" << itemconfigure(squares[i][j]) -Tk::fill("green");
	}else{
		".c" << itemconfigure(squares[i][j]) -Tk::fill("white");
	}
}

void clear(){
    for(int i = 0; i != arrayWidth; ++i){
        for(int j = 0; j != arrayHeight; ++j){
        	setCell(i, j, false);
        }
    }
}



int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_cmd;
	string input;

	Position start,goal;

	cout << "What is the intial location on a map x,y \n";
	cin >> start.posx >> start.posy;

	cout << "What is the goal location on a map x,y \n"
	cin >> goal.posx >> goal.posy;

	ros::Rate loop_rate(10);

	while(ros::ok()){
		

		if(state==-1){

		}
		cmd_vel_pub_.publish(base_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
