
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>
#include <random>

#include <unistd.h>

using namespace std;

string tf_prefix;

SDL_Renderer *renderer = NULL;
SDL_Window *win = NULL;
SDL_Texture *texture=NULL;
SDL_Surface *surface=NULL;
SDL_PixelFormat *fmt=NULL;
Uint32 temp, pixel;
Uint8 red, green, blue, alpha;
SDL_Color color;

geometry_msgs::Twist base_cmd;
nav_msgs::Odometry odom;

ros::Publisher cmd_vel_pub_;
ros::Publisher odom_pub;

std::random_device seeder;
std::mt19937 engine(seeder());

int arraywidth=820;
int arrayheight=700;

struct GVertex {
	int x;
	int y;
	int value;
	GVertex() :x(0), y(0){
		value=-1;
	}
	GVertex(int posx, int posy){
		x=posx;
		y=posy;
		value=0;
	}
};

struct GEdges{
	GEdges(){}
	GEdges(GVertex v,GVertex p){
		one.x = v.x;
		one.y = v.y;
		two.x = p.x;
		two.y = p.y;
	}
	GVertex one;
	GVertex two;
};

GVertex convertCoords(GVertex point);

struct Graph{
	Graph(){}
	Graph(pair<GVertex, GVertex> init, int k,int newdelta){
		maxvertices=k;
		start.x=init.first.x;
		start.y=init.first.y;
		goal.x=init.second.x;
		goal.y=init.second.y;
		vertices.push_back(init.first);
		robotVertices.push_back(convertCoords(init.first));
		delta = newdelta;
		//".c" << create(line, Point(800,800), Point(0,0)) -Tk::fill("black");
	}
	//int delta=20;
	//constint arraywidth=820;
	//constint arrayheight=700;
	//bool cells[820][700];
	int maxvertices;
	int delta;
	GVertex start;
	GVertex goal;
	vector<SDL_Point> renderpoints;
	vector<GEdges> edges;
	vector<GVertex> vertices;
	vector<GVertex> robotVertices;
}g;

struct Robot{
	Robot(){}
	Robot(double newh,int posx,int posy){
		h=newh;
		x=posx;
		y=posy;
	}
	double h;
	int x, y;
	GVertex g;
}robot;

void getInput();
void getRobotPosition();
float dist(GVertex a,GVertex b);

Uint32 getpixel(SDL_Surface *surface, int x, int y);
bool checkPixel(GVertex p);

vector<GVertex> Brensenham(int x,int y);
bool checkLine(GVertex pnear, GVertex prand);

GVertex nearestVertex(GVertex prand);
void add_vertex(GVertex pnear,GVertex pnew);

GVertex randConf();
vector<GVertex> newConf(vector<GVertex> prand);
bool RRT();

void turnRobot();
void moveTo();

void draw(vector<GVertex> prand,GVertex v);


void getInput(){
	SDL_Event e;
        while(SDL_PollEvent(&e)) {
        	switch(e.type){
        		case SDL_QUIT:
        			exit(1);
        			break;
        		case SDL_KEYDOWN:
        		case SDL_KEYUP:
        			switch(e.key.keysym.sym){
        				case SDLK_BACKSPACE:
        				case SDLK_ESCAPE:
        				case SDLK_q:
        					exit(1);
        			}
        			break;
        		default:
        			continue;
        	}
            
        }
}

void getRobotPosition(){
    tf::TransformListener listener;
    tf::StampedTransform transform;

    cerr << "GET ROBOT POS" << endl;

    try {
        string base_footprint_frame = tf::resolve(tf_prefix, "base_footprint");
	
        listener.waitForTransform("/odom", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
    	listener.lookupTransform("/odom", base_footprint_frame, ros::Time(0), transform);

       	robot=Robot(tf::getYaw(transform.getRotation()),transform.getOrigin().x(),transform.getOrigin().y());
       	cerr << "Robot ->" << " Heading: " << robot.h << " Robot pos x: " << robot.x << " Robot pos y: " << robot.y  << endl;
    }
    catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
    }
}

void setRobotPosition(){
    tf::TransformBroadcaster transformBroadcaster;
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped odom_trans;

    cerr << "SET ROBOT POS" << endl;

    try {
    	ros::Time current_time = ros::Time::now();
        string base_footprint_frame = tf::resolve(tf_prefix, "base_footprint");
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = g.start.x;
        odom_trans.transform.translation.y = g.start.y;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        
     odom.header.stamp = current_time;
     odom.header.frame_id = "odom";
   
     odom.pose.pose.position.x = g.start.x;
     odom.pose.pose.position.y = g.start.y;
     odom.pose.pose.position.z = 0.0;
     odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
   
     odom.child_frame_id = "base_link";
     odom.twist.twist.linear.x = 0;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = 0.0;
   
		cerr << "x: " << odom.pose.pose.position.x << "y: " <<  odom.pose.pose.position.y << endl;

		transformBroadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
       	//cerr << "Robot:" << "Heading: " << robot.h << "Robot pos x: " << robot.x << "Robot pos y: " << robot.y  << endl;
    }
    catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
    }
}

float dist(GVertex a,GVertex b){
	return sqrt(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
}

//from http://sdl.beuc.net/sdl.wiki/Pixel_Access
Uint32 getpixel(SDL_Surface *surface, int x, int y){
    int bpp = surface->format->BytesPerPixel;

    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        return *p;
        break;

    case 2:
        return *(Uint16 *)p;
        break;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
            return p[0] << 16 | p[1] << 8 | p[2];
        else
            return p[0] | p[1] << 8 | p[2] << 16;
        break;

    case 4:
        return *(Uint32 *)p;
        break;

    default:
        return 0;       
    }
}

//returns true if white
bool checkPixel(int x, int y){
	//cerr << "Check Pixel" << endl;
	SDL_LockSurface(surface);
	int pixel = getpixel(surface,x,y);
	SDL_UnlockSurface(surface);
	SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
	//fprintf(stderr, "Pixel info x: %d y: %d -> R: %d G: %d B: %d\n",x,y,red,green,blue);
	return (red==255 && green==255 && blue==255);
}

/* 	check for vertical line
	check for horizontal line 
	
	check if start is greater than end
	if false
		switch
	get points on line	

*/

//bresenham line algorithm from http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C
vector<GVertex> Bresenham(GVertex p0,GVertex p1) {
	vector<GVertex> v;
 
  int dx = abs(p1.x-p0.x), sx = p0.x<p1.x ? 1 : -1;
  int dy = abs(p1.y-p0.y), sy = p0.y<p1.y ? 1 : -1; 
  int err = (dx>dy ? dx : -dy)/2, e2;
 
  for(;;){
    //setPixel(x0,y0);
    v.push_back(GVertex(p0.x,p0.y));
    if (p0.x==p1.x && p0.y==p1.y) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; p0.x += sx; }
    if (e2 < dy) { err += dx; p0.y += sy; }
  }
  return v;
}


/*vector<GVertex> Bresenham(GVertex start, GVertex end){
cerr << "Bresenham line:" << endl;
	vector<GVertex> v;

	if(start.x==end.x){

		if(start.y<end.y){

			v.push_back(start);

			for(int y=start.y+1;y<=end.y;++y){
				v.push_back(GVertex(start.x,y));
			}

		}else{

			v.push_back(end);

			for(int y=end.y+1;y<=start.y;++y){
				v.push_back(GVertex(end.x,y));
			}

		}
	}else if(start.y==end.y){

		if(start.x<end.x){

			v.push_back(start);

			for(int x=start.x+1;x<=end.x;++x){
				v.push_back(GVertex(x,start.y));
			}

		}else{

			v.push_back(end);

			for(int y=end.y+1;y<=start.y;++y){
				v.push_back(GVertex(end.x,y));
			}

		}
	}else{

		if((start.x<end.x)&&(start.y<end.y)){
			v.push_back(start);

			int dx = end.x-start.x;
  			int dy = end.y-start.y;

  			int err = 2*dy - dx;
  			int y = start.y;

  			for(int x = start.x+1;x <= end.x; ++x){
    			err = err + (2*dy);
    			if(err > 0){
      				y = y+1;
      				err = err - (2*dx);
      				v.push_back(GVertex(x,y));
  				}
  			}
  		}else{
  			v.push_back(end);

			int dx = start.x-end.x;
  			int dy = start.y-end.y;

  			int err = 2*dy - dx;
  			int y = end.y;

  			for(int x = end.x+1;x <= start.x; ++x){
    			err = err + (2*dy);
    			if(err > 0){
      				y = y+1;
      				err = err - (2*dx);
      				v.push_back(GVertex(x,y));
  				}
  			}
  		}
	}


	for(int i=0;i<v.size();++i){
		cerr << "Point on line: "<< i << " x: " << v[i].x << " y: " << v[i].y <<  endl;
	}

  return v;
}
*/

//true if line is white
bool checkLine(GVertex pnear, GVertex prand){
	//cerr << "CheckLine" << endl;

	vector<GVertex> v(Bresenham(pnear,prand));
		for(int i=0;i<v.size();++i){
			if(!checkPixel(v[i].x,v[i].y)){
				//cerr << "false" << endl;
				return false;
			}
		}
		//cerr << "true" << endl;
		return true;
}


//nearest vertex of prand to goal
/*GVertex nearestVertex(GVertex prand){
	cerr << "Nearest Vertex" << endl;
	GVertex temp=GVertex(g.vertices[0]);
	for(int i=0;i<g.vertices.size();++i){
		if(dist(prand,g.goal)<dist(temp,prand)){
			temp.x=g.vertices[i].x;
			temp.y=g.vertices[i].y;
		}
	}
	return temp;
}*/

bool nearestVertex(GVertex prand,GVertex nearest){
	if(dist(prand,g.goal)<=dist(nearest,g.goal)){
		return true;
	}
	return false;
}

bool equal(GVertex p1,GVertex p2){
	return (p1.x==p2.x&&p1.y==p2.y);
}

GVertex getNearestVertex(vector<GVertex> prand){
	//cerr << "Nearest Vertex" << endl;
	GVertex temp=GVertex(prand.back().x,prand.back().y);
	while(!prand.empty()){
    	/*for(int e=0;e<prand.size();++e){
       		if(SDL_RenderDrawLine(renderer,g.vertices.back().x,g.vertices.back().y,prand[e].x,prand[e].y)!=0){
        		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        		exit(1);
    		}
    	}*/
   		if(!checkLine(g.vertices.back(),prand.back())){
    		//temp=GVertex(prand.end()-1;
    		if(equal(temp,prand.back())){
    			prand.pop_back();
    			temp=GVertex(prand.back().x,prand.back().y);
    		}else{
    			prand.pop_back();
    		}
    	}else if(nearestVertex(prand.back(),temp)){
			temp=GVertex(prand.back().x,prand.back().y);
			prand.pop_back();
		}else{
			prand.pop_back();
		}
		draw(prand,temp);
		getInput();
	}
	return temp;
}

GVertex convertCoords(GVertex point){
	return GVertex(point.x-((1/2)*arraywidth),point.y+((1/2)*arrayheight));
}

void add_vertex(GVertex pnear,GVertex pnew){
	//cerr << "Add Vertex" << endl;
	//SDL_Point p={pnew.x,pnew.y};
	//SDL_Point p={pnear.x,pnear.y};
	//g.renderpoints.push_back(p);
	g.robotVertices.push_back(convertCoords(pnew));
	g.vertices.push_back(pnew);
	g.edges.push_back(GEdges(pnear,pnew));
}

vector<GVertex> rand_conf(){
	//cerr << "Random Configuration" << endl;

	int x,y;
	//GVertex p;

	//int maxx=robpos.x+g.delta,minx=robpos.x-delta;

	//robpos=getRobotPosition();

	vector<GVertex> v;

	//std::uniform_int_distribution<int> distx;
	//std::uniform_int_distribution<int> disty;

	//do{
		//rand()%(max-min + 1) + min;
	//uniform_int_distribution<int> distx(robot.x-g.delta,robot.x+g.delta);
	//uniform_int_distribution<int> disty(robot.y-g.delta,robot.y+g.delta);
	//uniform_int_distribution<int> distx(0,arraywidth);
	//uniform_int_distribution<int> disty(0,arrayheight);
	uniform_int_distribution<int> distx(g.vertices.back().x-g.delta,g.vertices.back().x+g.delta);
	uniform_int_distribution<int> disty(g.vertices.back().y-g.delta,g.vertices.back().y+g.delta);
	for(int i=0;i<g.maxvertices;++i){
		do{
			x = distx(engine);
			y = disty(engine);
			if(x<=0){
				x=0;
			}
			if(x>=arraywidth){
				x=arraywidth;
			}
			if(y<=0){
				y=0;
			}

			if(y>=arrayheight){
				y=arrayheight;
			}
			SDL_LockSurface(surface);
			pixel = getpixel(surface,x,y);
			SDL_UnlockSurface(surface);
			SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
			/*if(checkLine(g.vertices.back(),GVertex(x,y))){
				break;
			}*/
			getInput();
			//fprintf(stderr, "Pixel %d x: %d y: %d Color -> R: %d G: %d B: %d A: %d \n",pixel,x,y,red,green,blue,alpha);
		}while(red==0&&green==0&&blue==0);

		v.push_back(GVertex(x,y));
	}
		//cerr << "Pixel Color -> R: "<< (int)red << " G: " << (int)green << " B: " << (int)blue <<  " A: " << (int)alpha << endl;
		//fprintf(stderr, "Pixel %d x: %d y: %d Color -> R: %d G: %d B: %d A: %d \n",pixel,x,y,red,green,blue,alpha);
	//}//while(red!=0&&green!=0&&blue!=0);

	//return nearestVertex(GVertex(x,y));
	//return make_pair(p,GVertex(x,y));
	//return GVertex(distx,disty);
	return v;
}


/*vector<GVertex> newConf(vector<GVertex> prand){
	cerr << "NEW Configuration" << endl;
	GVertex temp=GVertex(prand.back());
	vector<GVertex> v(1);
	while(!prand.empty()){
		if(checkLine(g.vertices.back(),prand.back()) && nearestVertex(prand.back(),temp)){
			//prand.erase(prand.end());
			v[0].x=temp.x;
			v[0].y=temp.y;
			prand.pop_back();
			//draw(prand);
		}else{
			//v.push_back(prand.back());
			//draw(v);
			prand.pop_back();
		}
		draw(prand,temp);
		getInput();

		/*if(nearestVertex(prand.back(),temp)){
			temp.x=prand.back().x;
			temp.y=prand.back().y;
			prand.pop_back();
		}else{
			prand.pop_back();
		}
		
	}
	return v;
}*/

/*if(checkLine(g.vertices.back(),GVertex(x,y))){
				break;
			}*/

GVertex nearGoal(GVertex pnew){
	//cerr << "NEAR GOAL" << endl;
	if(dist(g.vertices.back(),g.goal)<=g.delta && checkLine(g.vertices.back(),g.goal) && (dist(pnew,g.goal)>dist(g.vertices.back(),g.goal))){
		return g.goal;
	}
	return pnew;
}

void draw(vector<GVertex> prand,GVertex v){
	//cerr << "DRAW" << endl;
	SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    for(int e=0;e<g.edges.size();++e){
       	if(SDL_RenderDrawLine(renderer,g.edges[e].one.x,g.edges[e].one.y,g.edges[e].two.x,g.edges[e].two.y)!=0){
       		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
       		exit(1);
    	}
    }

    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    if(!prand.empty()){
    	for(int k=0;k<prand.size();++k){
       		if(SDL_RenderDrawLine(renderer,g.vertices.back().x,g.vertices.back().y,prand[k].x,prand[k].y)!=0){
       			fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
       			exit(1);
    		}
    	}
    }

    if(v.value!=-1){
       	if(SDL_RenderDrawLine(renderer,g.vertices.back().x,g.vertices.back().y,v.x,v.y)!=0){
       		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
       		exit(1);
    	}
    }

    SDL_RenderSetScale( renderer, 2, 2 );
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawPoint(renderer,g.goal.x/2,g.goal.y/2);
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderDrawPoint(renderer,g.start.x/2,g.start.y/2);
 	SDL_RenderSetScale( renderer, 1, 1);
    SDL_RenderPresent(renderer);
    SDL_Delay(200);
}

bool RRT(){
	GVertex pnear,pnew;
	
	//for(int i = 1; i <= g.maxvertices; ++i){
		//cerr << "Iteration: " << i << endl;
	vector<GVertex> prand=rand_conf();
	//vector<GVertex> v=newConf(prand);
	//if(!v.empty()){
		pnew=getNearestVertex(prand);
	if(pnew.x!=-1 || pnew.y!=-1){
		pnew=nearGoal(pnew);
		add_vertex(g.vertices.back(),pnew);


		//SDL_RenderPresent(renderer);
		//cerr << "List of points: " << i << endl;
		//cerr << "NEW point: " << endl;
		//cerr << "X: " << g.vertices.back().x << " Y: " << g.vertices.back().y << endl;
		//for(int k=0;k<g.renderpoints.size();k++){
			//cerr << "X: " << g.renderpoints[k].x << " Y: " << g.renderpoints[k].y << endl;
		//}

		//SDL_RenderClear(renderer);
    	//SDL_RenderCopy(renderer, texture, NULL, NULL);
		/*if(SDL_RenderDrawLines(renderer,&g.renderpoints[0],g.renderpoints.size())!=0){
        	fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        	exit(1);
    	}*/

        //for(int e=0;e<g.edges.size();++e){
       	//	if(SDL_RenderDrawLine(renderer,g.edges[e].one.x,g.edges[e].one.y,g.edges[e].two.x,g.edges[e].two.y)!=0){
        //		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        //		exit(1);
    	//	}
    	//}

    	//SDL_RenderSetScale( renderer, 3, 3 );

    	//SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    	//SDL_RenderDrawPoint(renderer,g.goal.x,g.goal.y);
    	//SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    	//SDL_RenderDrawPoint(renderer,g.start.x,g.start.y);
    	//SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    	//SDL_RenderSetScale( renderer, 1, 1 );

    	//SDL_RenderPresent(renderer);
    	//SDL_RenderPresent(renderer);

    	/*if((pnew.x==g.start.x)&&(pnew.y==g.start.y)){
    		cerr << "Graph complete" << endl;
    		while(true){
    			getInput();
    		}
    		//SDL_RenderPresent(renderer);
    	}*/
    	//draw(prand);
    		getInput();
    return true;
   }else{
    	return false;
   }
    	
    	//SDL_Delay(500);
	//return g;
}

bool near(GVertex o, GVertex t){
	return ((o.x-.05<=t.x && t.x<=o.x+.05)&&(o.y-.05<=t.y && t.y<=o.y+.05));
}

double bearing(){
	return atan2(g.robotVertices.back().y-robot.y,g.robotVertices.back().x-robot.x);
}

bool isFacing(){
	double num=bearing();
	return ((num-.05 <= robot.h) && (robot.h <= num+.05));
}

void moveTo(){
	while(!near(GVertex(robot.x,robot.y),g.robotVertices.back())){
		base_cmd.linear.x=.25;
		draw(vector<GVertex>(),GVertex());
		getInput();
		cmd_vel_pub_.publish(base_cmd);
	}
}

/*bool reachGoal(){
	return (near(GVertex(robot.x,robot.y),g.goal));
}*/

bool reachGoal(){
	return (near(g.vertices.back(),g.goal));
}

void turnRobot(){
	cerr << "TURN" << endl;
	while(!isFacing()){
		if(bearing()-robot.h>0){
			base_cmd.angular.z=-0.25;
		}else{
			base_cmd.angular.z=0.25;
		}
		draw(vector<GVertex>(),GVertex());
		getInput();
		cmd_vel_pub_.publish(base_cmd);
	}
}	

int main(int argc, char **argv){

	ros::init(argc, argv, "assignment2");
	ros::NodeHandle nh_;
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
	nh_.getParam("tf_prefix", tf_prefix);
	string input;
	int state;

	
	int x,y;

	if(SDL_Init(SDL_INIT_EVERYTHING)!=0){
        fprintf(stderr, "Error: Unable to init SDL: %s\n", SDL_GetError());
        exit(1);
    }

    int image_flags = IMG_INIT_PNG;
	if(IMG_Init(image_flags) != image_flags){
    	cout << "Error: " << IMG_GetError() << endl;
	}

    win = SDL_CreateWindow("RRT GRAPH", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,arraywidth, arrayheight,SDL_WINDOW_SHOWN);

    if(!win){
       	fprintf(stderr, "Unable to initilize SDL2: %s\n", SDL_GetError());
       	SDL_Quit();
       	exit(1);
    }

    char buff[255];
    getcwd(buff,255);
    cout << buff << endl;
    //string filepath = "/home/csguest/catkin_ros/src/assignment2/Stage/bitmaps/autolab.png";
    string filepath = "/src/assignment2/Stage/bitmaps/autolab.png";
    string fullpath=string(buff).append(filepath);

    surface=IMG_Load(fullpath.c_str());
    //surface = SDL_LoadBMP(../Stage/bitmaps/autolab.png);
    //surface = CreateRGBSurfaceFrom(,width,height,32,0,0,0,0);

    if(!surface){
    	fprintf(stderr, "Create Surface failed: %s\n", SDL_GetError());
        exit(-1);
    }



    //SDL_LockSurface(surface);

    /*
    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, 1024, 768);

    if(!texture){
    	fprintf(stderr, "Unable to create texture SDL2: %s\n", SDL_GetError());
       	SDL_Quit();
       	exit(1);
    }*/

    renderer = SDL_CreateRenderer(win, -1, 0);
    texture=SDL_CreateTextureFromSurface(renderer, surface);

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);

   	//SDL_RenderFillRect( renderer, &r );
    //Render the rect to the screen
    //SDL_RenderPresent(renderer);
    //SDL_SetRenderDrawColor(SDL_Renderer* renderer,0,0,0,255)

	cout << "What is the intial location on a map x,y" << endl;
	do{
		cerr << "Make sure your position is not inside a obsticle" << endl;
		cin >> x >> y;
	}while(!checkPixel(x,y));

	GVertex start=GVertex(x,y);

	cout << "What is the goal location on a map x,y" << endl;
	do{
		cerr << "Make sure your position is not inside a obsticle" << endl;
		cin >> x >> y;
	}while(!checkPixel(x,y));

	GVertex goal=GVertex(x,y);

	robot.g=convertCoords(goal);

	ros::Rate loop_rate(10);
	pair<GVertex,GVertex> ginit = make_pair(start,goal);

	//RRT(ginit,10);

	g=Graph(ginit,10,40);

	//s/etRobotPosition();
   
    //odom_pub.publish(odom);

	while(ros::ok()){
		
		cmd_vel_pub_.publish(base_cmd);
//odom_pub.publish(odom);
		if(RRT()){
		//getRobotPosition();
		//turnRobot();
		//moveTo();
			if(reachGoal()){
				while(true){
					draw(vector<GVertex>(),GVertex());
					getInput();
				}
			}
		}
		//draw();
		getInput();

		ros::spinOnce();
		loop_rate.sleep();
	}

	SDL_DestroyRenderer(renderer);
	SDL_DestroyTexture(texture);
	SDL_FreeSurface(surface);
	SDL_DestroyWindow(win);
    SDL_Quit();

	return 0;
}
