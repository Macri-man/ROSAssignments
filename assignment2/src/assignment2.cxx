
#include <ros/ros.h>
#include <tf/transform_listener.h>
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
#include <ctime>
#include <random>

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


auto const seed = std::random_device()();          

std::mt19937 rng;                  

void initialize(){
  rng.seed(seed);
}


struct GVertex {
	int x;
	int y;
	GVertex() :x(0), y(0){
	}
	GVertex(int posx, int posy){
		x=posx;
		y=posy;
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

struct GNodes{
	GNodes(){}
	GNodes(GVertex v){
		vertex.x=v.x;
		vertex.y=v.y;

	}
	GNodes(GVertex point){
		vertex.x=point.x;
		vertex.y=point.y;
	}

	GNodes *parent;
	vector<GNodes> children;
	GVertex vertex;
};

struct Graph{
	Graph(){}
	Graph(pair<GVertex, GVertex> init, int k,int newdelta){
		maxvertices=k;
		start.x=init.first.x;
		start.y=init.first.y;
		goal.x=init.second.x;
		goal.y=init.second.y;
		vertices.push_back(init.first);
		delta = newdelta;
		root=GNodes(init.first);
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
	GNodes *root;
}g;

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
GVertex newConf();
Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta);

GNodes convertCoords();
void turnRobot();
void goStraight();

void removeChild();
void addChild();


void removeChild(GNodes *parent){
	*parent.children.pop_back();
}

void addChild(GNodes *parent,GNodes *child){
	*parent.children.push_back(&child); 
}



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
        					exit(1);
        			}
        			break;
        		default:
        			continue;
        	}
            
        }
}

GVertex getRobotPosition(){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    GVertex pos;

    try {
        string base_footprint_frame = tf::resolve(tf_prefix, "base_footprint");
	
        listener.waitForTransform("/odom", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/odom", base_footprint_frame, ros::Time(0), transform);

        GVertex(transform.getOrigin().x(),transform.getOrigin().y());
    }
    catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
    }
    return pos;
}

float dist(GVertex a,GVertex b){
	return sqrt(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
}


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
	SDL_LockSurface(surface);
	int pixel = getpixel(surface,x,y);
	SDL_UnlockSurface(surface);
	SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
	fprintf(stderr, "Pixel info x: %d y: %d -> R: %d  G: %d B: %d\n",x,y,red,green,blue);
	return (red==255 && green==255 && blue==255);
}

/* 	check for vertical line
	check for horizontal line 
	
	check if start is greater than end
	if false
		switch
	get points on line	

*/


vector<GVertex> Bresenham(GVertex start, GVertex end){
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


//true if line is white
bool checkLine(GVertex pnear, GVertex prand){
	cerr << "CheckLine" << endl;

	vector<GVertex> v(Bresenham(pnear,prand));
		for(int i=0;i<v.size();++i){
			if(!checkPixel(v[i].x,v[i].y)){
				return false;
			}
		}
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
	cerr << "Nearest Vertex" << endl;
	if(dist(prand,g.goal)<=dist(nearest,g.goal)){
		return true;
	}
	return false;
}

GVertex getNearestVertex(vector<GVertex> prand){
	GVertex temp=GVertex(prand.back());
	while(!prand.empty()){
		SDL_RenderClear(renderer);
    	SDL_RenderCopy(renderer, texture, NULL, NULL);
    	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    	for(int e=0;e<prand.size();++e){
       		if(SDL_RenderDrawLine(renderer,g.vertices.back().x,g.vertices.back().y,prand[e].x,prand[e].y)!=0){
        		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        		exit(1);
    		}
    	}
		if(nearestVertex(prand.back(),temp)){
			temp.x=prand.back().x;
			temp.y=prand.back().y;
			prand.pop_back();
		}else{
			prand.pop_back();
		}
		SDL_RenderPresent(renderer);
	}
	return temp;
}

void add_vertex(GVertex pnear,GVertex pnew){
	cerr << "Add Vertex" << endl;
	//SDL_Point p={pnew.x,pnew.y};
	//SDL_Point p={pnear.x,pnear.y};
	//g.renderpoints.push_back(p);
	g.vertices.push_back(pnew);
	g.edges.push_back(GEdges(pnear,pnew));
}

vector<GVertex> rand_conf(){
	cerr << "Random Configuration" << endl;

	//int x,y;
	//GVertex p;

	//int maxx=robpos.x+g.delta,minx=robpos.x-delta;

	//robpos=getRobotPosition();

	vector<GVertex> v;

	std::uniform_int_distribution<uint32_t> distx;
	std::uniform_int_distribution<uint32_t> disty;

	//do{
		//rand()%(max-min + 1) + min;
		//distx = uniform_int_distribution<uint32_t>(convertCoords(robpos).x-20,convertCoords(robpos).x+20);
		//disty = uniform_int_distribution<uint32_t>(convertCoords(robpos).y-20,convertCoords(robpos).y+20);
	for(int i=0;i<30;++i){
		distx = uniform_int_distribution<uint32_t>(g.vertices.back().x-20,g.vertices.back().x-+20);
		disty = uniform_int_distribution<uint32_t>(g.vertices.back().y-20,g.vertices.back().y-+20);
		do{
			SDL_LockSurface(surface);
			pixel = getpixel(surface,distx,disty);
			SDL_UnlockSurface(surface);
			SDL_GetRGBA(pixel,surface->format,&red,&green,&blue,&alpha);
			if(checkLine(g.vertices.back(),GVertex(x,y))){
				break;
			}
		}while(red!=0&&green!=0&&blue!=0);

		v.push_back(GVertex(distx,disty));
	}
		//cerr << "Pixel Color -> R: "<< (int)red << " G: " << (int)green << " B: " << (int)blue <<  " A: " << (int)alpha << endl;
		//fprintf(stderr, "Pixel %d x: %d y: %d Color -> R: %d G: %d B: %d A: %d \n",pixel,x,y,red,green,blue,alpha);
	//}//while(red!=0&&green!=0&&blue!=0);

	//return nearestVertex(GVertex(x,y));
	//return make_pair(p,GVertex(x,y));
	//return GVertex(distx,disty);
	return v;
}

GVertex new_conf(GVertex pnear,GVertex prand){
	cerr << "NEW Configuration" << endl;
	
}

Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta){
	GVertex prand,pnear,pnew;
	g=Graph(qinit,kvertices,delta);
	//for(int i = 1; i <= g.maxvertices; ++i){
		cerr << "Iteration: " << i << endl;
		vector<GVertex> prand=rand_conf();
		pnew=getNearestVertex(prand);
		//pnew=new_conf();
		add_vertex(g.vertices.back(),pnew);


		//SDL_RenderPresent(renderer);
		//cerr << "List of points: " << i << endl;
		cerr << "NEW point: " << i << endl;
		cerr << "X: " << g.vertices[g.vertices.size()-1].x << " Y: " << g.vertices[g.vertices.size()-1].y << endl;
		//for(int k=0;k<g.renderpoints.size();k++){
			//cerr << "X: " << g.renderpoints[k].x << " Y: " << g.renderpoints[k].y << endl;
		//}

		SDL_RenderClear(renderer);
    	SDL_RenderCopy(renderer, texture, NULL, NULL);
		/*if(SDL_RenderDrawLines(renderer,&g.renderpoints[0],g.renderpoints.size())!=0){
        	fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        	exit(1);
    	}*/

        for(int e=0;e<g.edges.size();++e){
       		if(SDL_RenderDrawLine(renderer,g.edges[e].one.x,g.edges[e].one.y,g.edges[e].two.x,g.edges[e].two.y)!=0){
        		fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        		exit(1);
    		}
    	}

    	//SDL_RenderSetScale( renderer, 3, 3 );

    	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    	SDL_RenderDrawPoint(renderer,g.goal.x,g.goal.y);
    	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    	SDL_RenderDrawPoint(renderer,g.start.x,g.start.y);
    	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    	//SDL_RenderSetScale( renderer, 1, 1 );

    	SDL_RenderPresent(renderer);
    	//SDL_RenderPresent(renderer);

    	if((pnew.x==g.start.x)&&(pnew.y==g.start.y)){
    		cerr << "Graph complete" << endl;
    		while(true){
    			getInput();
    		}
    		//SDL_RenderPresent(renderer);
    	}

    	getInput();
    	SDL_Delay(500);

	//}
	return g;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	nh_.getParam("tf_prefix", tf_prefix);
	geometry_msgs::Twist base_cmd;
	string input;
	int state;

	int x,y;
	srand (time(NULL));

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

    string filepath = "/home/csguest/catkin_ros/src/Stage/bitmaps/autolab.png";

    surface=IMG_Load(filepath.c_str());
    //surface = SDL_LoadBMP(../Stage/bitmaps/autolab.png);
    //surface = CreateRGBSurfaceFrom(,width,height,32,0,0,0,0);

    if(!surface){
    	fprintf(stderr, "Create Surface failed: %s\n", SDL_GetError());
        exit(1);
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
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

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

	ros::Rate loop_rate(10);
	pair<GVertex,GVertex> ginit = make_pair(start,goal);

	RRT(ginit,3000,10);

	while(ros::ok()){
		
		cmd_vel_pub_.publish(base_cmd);
		

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
