
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

using namespace std;

const int arraywidth=820;
const int arrayheight=700;
const int state = 0;

SDL_Renderer *renderer = NULL;
SDL_Window *win = NULL;
SDL_Texture *texture=NULL;
SDL_Surface *surface=NULL;
SDL_PixelFormat *fmt=NULL;


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
		one = v;
		two = p;
	}
	GVertex one;
	GVertex two;
};


float dist(GVertex a, GVertex b){
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
}g;

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

GVertex nearest_vetex(GVertex prand){
	GVertex temp=GVertex(g.vertices[0]);
	for(int i=0;i<g.vertices.size();++i){
		if(dist(prand,g.vertices[i])<dist(prand,temp)){
			temp.x=g.vertices[i].x;
			temp.y=g.vertices[i].y;
		}
	}
	return temp;
}

GVertex new_conf(GVertex pnear,GVertex prand,int delta){
	if(dist(pnear,prand)< delta){
		return prand;
	}else{
		float theta = atan2(prand.x-pnear.x,prand.y-pnear.y);
		return GVertex(pnear.x+delta*cos(theta),pnear.y+delta*sin(theta));
	}
}

GVertex rand_conf(){
	cerr << "random configuration" << endl;
	srand (time(NULL));
	int x = rand() % (arraywidth) + 1;
	int y = rand() % (arrayheight) + 1;
	return GVertex(x,y);
}

void add_vertex(GVertex pnear,GVertex pnew){
	cerr << "add vertex" << endl;
	//g.lines.push_back(".c" << create(line, Point(pnear.x,pnear.y), Point(pnew.x,pnew.y)) -Tk::fill("black"));
	SDL_Point p={pnew.x,pnew.y};
	g.renderpoints.push_back(p);
	g.vertices.push_back(pnew);
	g.edges.push_back(GEdges(pnear,pnew));
}

Graph RRT(pair<GVertex,GVertex> qinit,int kvertices,int delta){
	GVertex prand,pnear,pnew;
	g=Graph(qinit,kvertices,delta);
	for(int i = 1; i <= g.maxvertices; ++i){
		cerr << "Iteration:" << i << endl;
		prand=rand_conf();
		pnear=nearest_vetex(prand);
		pnew=new_conf(pnear,prand,g.delta);
		add_vertex(pnear,pnew);

		SDL_RenderPresent(renderer);
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

		for(int i=0;i<g.renderpoints.size();i++){
			cout << g.renderpoints[i].x << g.renderpoints[i].y;
		}

		if(SDL_RenderDrawLines(renderer,&g.renderpoints[0],g.renderpoints.size())!=0){
        	fprintf(stderr, "Error: Unable to render lines: %s\n", SDL_GetError());
        	exit(1);
    	}

    	SDL_RenderPresent(renderer);
    	SDL_Delay(500);

	}
	return g;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stage");
	ros::NodeHandle nh_;
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_cmd;
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

    string filepath = "/home/csguest/catkin_ros/src/Stage/bitmaps/autolab.png";

    surface=IMG_Load(filepath.c_str());
    //surface = SDL_LoadBMP(../Stage/bitmaps/autolab.png);
    //surface = CreateRGBSurfaceFrom(,width,height,32,0,0,0,0);

    if(!surface){
    	fprintf(stderr, "Create Surface failed: %s\n", SDL_GetError());
        exit(1);
    }

    SDL_LockSurface(surface);

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


    // Render rect
   // SDL_RenderFillRect( renderer, &r );

    // Render the rect to the screen
    //SDL_RenderPresent(renderer);


	cout << "What is the intial location on a map x,y \n";
	cin >> x >> y;

	GVertex start=GVertex(x,y);

	cout << "What is the goal location on a map x,y \n";
	cin >> x >> y;

	GVertex goal=GVertex(x,y);

	ros::Rate loop_rate(10);

	pair<GVertex,GVertex> stuff = make_pair(start,goal);

	/*try{    
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
    }*/

	while(ros::ok()){
		
		cmd_vel_pub_.publish(base_cmd);
		RRT(stuff,10,20);
		SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }

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
