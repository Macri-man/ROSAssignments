
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "cpptk.h"
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>
#include <ctime>

using namespace Tk;
using namespace std;

const int arraywidth=820;
const int arrayheight=700;
const int state = 0;

struct GVertex {
		GVertex(){
			x=0;
			y=0;
		}
		GVertex(int posx, int posy){
			x=posx;
			y=posy;
		}
		int x;
		int y;
};

struct GEdges{
	GEdges(){}
	GEdges(GVertex v,GVertex p){
		one = v;
		two = p;
	}
	GVertex one;
	GVertex two;
};


float distance(GVertex a, GVertex b){
	return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

struct Graph{
	Graph(){}
	Graph(pair<GVertex, GVertex> init, int k,int delta){
		maxvertices=k;
		start.x=init.first.x;
		start.y=init.first.y;
		goal.x=init.second.x;
		goal.y=init.second.y;
		vertices.push_back(init.second);
		".c" << create(line, Point(800,800), Point(0,0)) -Tk::fill("black");
	}
	//int delta=20;
	//constint arraywidth=820;
	//constint arrayheight=700;
	//bool cells[820][700];
	int maxvertices;
	int delta;
	GVertex start;
	GVertex goal;
	vector<string> lines;
	vector<GEdges> edges;
	vector<GVertex> vertices;
};

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

  /* refrence def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)
*/

GVertex nearest_vetex(GVertex prand,Graph g){
	GVertex temp=GVertex(g.vertices[0]);
	for(int i=0;i<g.vertices.size();++i){
		/*if(distance(prand,)<distance(prand,temp)){
			temp.x=g.vertices[i].x;
			temp.y=g.vertices[i].y;
		}*/
	}
	return temp;
}

GVertex new_conf(GVertex pnear,GVertex prand,int delta){
	/*if(distance(pnear,prand)< delta){
		return prand;
	}else{
		float theta = atan2(prand.x-pnear.x,prand.y-pnear.y);
		return GVertex(pnear.x+delta*cos(theta),pnear.y+delta*sin(theta));
	}*/
}

GVertex rand_conf(){
	cerr << "random configuration" << endl;
	srand (time(NULL));
	int x = rand() % arraywidth + 1;
	int y = rand() % arrayheight + 1;
	return GVertex(x,y);
}

void add_vertex(GVertex pnear,GVertex pnew,Graph g){
	cerr << "add vertex" << endl;
	g.lines.push_back(".c" << create(line, Point(pnear.x,pnear.y), Point(pnew.x,pnew.y)) -Tk::fill("black"));
	g.vertices.push_back(pnew);
	g.edges.push_back(GEdges(pnear,pnew));
}

Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta){
	GVertex prand,pnear,pnew;
	Graph g=Graph(qinit,kvertices,delta);
	for(int i = 1; i < g.maxvertices; ++i){
		cerr << "Iteration:" << i << endl;
		prand=rand_conf();
		pnear=nearest_vetex(prand,g);
		pnew=new_conf(pnear,prand,g.delta);
		add_vertex(pnear,pnew,g);
	}
	return g;
}

void setCell(int i,int j,bool state){
	//::cells[i][j]=state;
	if(state){
		//".c" << itemconfigure(squares[i][j]) -Tk::fill("green");
	}else{
		//".c" << itemconfigure(squares[i][j]) -Tk::fill("white");
	}
}

void clear(){
  //  for(int i = 0; i != arrayWidth; ++i){
     //   for(int j = 0; j != arrayHeight; ++j){
        	//setCell(i, j, false);
       // }
  //  }
}

void makeMap(GVertex start,GVertex goal){

}

void addline(GVertex p1,GVertex p2){
	//lines.push_back(".c" << create(line, p1, p2) -Tk::fill("black");
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_cmd;
	string input;
	int state;

	int x,y;

	cout << "What is the intial location on a map x,y \n";
	cin >> x >> y;

	GVertex start=GVertex(x,y);

	cout << "What is the goal location on a map x,y \n";
	cin >> x >> y;

	GVertex goal=GVertex(x,y);

	ros::Rate loop_rate(10);

	 pair<GVertex,GVertex> stuff = make_pair(start,goal);

	try{    
        init(argv[0]);
        frame(".f") -relief(raised) -borderwidth(1);         
        pack(canvas(".c") -background("white") -width(arraywidth) -height(arrayheight));

        update();
        cerr << "start";
        RRT(stuff,5000,7);
        wm(resizable, ".", false, false);
        runEventLoop();
    }catch (exception const &e){
        cerr << "Error: " << e.what() << '\n';
    }

   

    //RRT(stuff,5000,7);

	/*while(ros::ok()){
		
		cmd_vel_pub_.publish(base_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}*/
	return 0;
}
